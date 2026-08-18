#!/usr/bin/env python3
"""Collect diverse checkerboard views from the Pi stream and calibrate the C920."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path

import av
import cv2
import numpy as np


@dataclass
class CalibrationView:
    frame_index: int
    image: np.ndarray
    corners: np.ndarray
    feature: np.ndarray
    area_ratio: float
    blur_score: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--format", default="mpegts")
    parser.add_argument("--pattern-cols", type=int, default=9)
    parser.add_argument("--pattern-rows", type=int, default=6)
    parser.add_argument("--min-views", type=int, default=20)
    parser.add_argument("--max-views", type=int, default=32)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--diversity-threshold", type=float, default=0.075)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--name", required=True)
    return parser.parse_args()


def view_feature(corners: np.ndarray, width: int, height: int, cols: int, rows: int):
    points = corners.reshape(-1, 2)
    centroid = points.mean(axis=0)
    hull = cv2.convexHull(points.astype(np.float32))
    area_ratio = float(cv2.contourArea(hull) / (width * height))

    horizontal = points[cols - 1] - points[0]
    vertical = points[(rows - 1) * cols] - points[0]
    angle = math.atan2(float(horizontal[1]), float(horizontal[0]))
    top = np.linalg.norm(points[cols - 1] - points[0])
    bottom = np.linalg.norm(points[-1] - points[-cols])
    left = np.linalg.norm(points[(rows - 1) * cols] - points[0])
    right = np.linalg.norm(points[-1] - points[cols - 1])
    perspective_x = math.log(max(top, 1.0) / max(bottom, 1.0))
    perspective_y = math.log(max(left, 1.0) / max(right, 1.0))

    feature = np.array(
        [
            centroid[0] / width,
            centroid[1] / height,
            1.5 * math.sqrt(max(area_ratio, 0.0)),
            0.35 * math.sin(angle),
            0.35 * math.cos(angle),
            0.30 * perspective_x,
            0.30 * perspective_y,
        ],
        dtype=np.float64,
    )
    return feature, area_ratio


def guidance(views: list[CalibrationView], width: int, height: int) -> str:
    if not views:
        return "point the camera at the complete checkerboard"
    centroids = np.array([view.corners.reshape(-1, 2).mean(axis=0) for view in views])
    span = np.ptp(centroids, axis=0) / np.array([width, height])
    areas = np.array([view.area_ratio for view in views])
    if span[0] < 0.35:
        return "move the camera farther left and right"
    if span[1] < 0.25:
        return "move the camera higher and lower"
    if float(np.ptp(np.sqrt(areas))) < 0.10:
        return "add both close and far views"
    return "add tilted corner views while keeping the whole board visible"


def calibrate(
    views: list[CalibrationView], pattern_size: tuple[int, int], image_size: tuple[int, int]
):
    cols, rows = pattern_size
    object_template = np.zeros((cols * rows, 3), dtype=np.float32)
    object_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    kept = list(range(len(views)))

    for _ in range(3):
        object_points = [object_template.copy() for _ in kept]
        image_points = [views[index].corners.astype(np.float32) for index in kept]
        rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
        errors = []
        for object_point, image_point, rvec, tvec in zip(
            object_points, image_points, rvecs, tvecs
        ):
            projected, _ = cv2.projectPoints(
                object_point, rvec, tvec, camera_matrix, distortion
            )
            delta = image_point.reshape(-1, 2) - projected.reshape(-1, 2)
            errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))

        error_array = np.asarray(errors)
        median = float(np.median(error_array))
        mad = float(np.median(np.abs(error_array - median)))
        cutoff = max(0.60, median + 3.0 * 1.4826 * max(mad, 0.02))
        rejected_positions = [
            position for position, error in enumerate(error_array) if error > cutoff
        ]
        if not rejected_positions or len(kept) - len(rejected_positions) < 20:
            return rms, camera_matrix, distortion, errors, kept
        rejected_set = set(rejected_positions)
        kept = [index for position, index in enumerate(kept) if position not in rejected_set]

    raise RuntimeError("unreachable calibration loop")


def main() -> int:
    args = parse_args()
    if args.min_views < 12 or args.max_views < args.min_views:
        raise ValueError("Require 12 <= min-views <= max-views")

    run_dir = args.output_dir / args.name
    accepted_dir = run_dir / "accepted"
    accepted_dir.mkdir(parents=True, exist_ok=True)
    pattern_size = (args.pattern_cols, args.pattern_rows)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    views: list[CalibrationView] = []
    started_at = time.monotonic()
    last_status_at = 0.0
    frame_index = 0
    detection_count = 0
    image_size: tuple[int, int] | None = None

    connection = None
    for _ in range(50):
        try:
            connection = socket.create_connection((args.host, args.port), timeout=1.0)
            break
        except OSError:
            time.sleep(0.2)
    if connection is None:
        raise RuntimeError("Camera stream did not become reachable within 10 seconds")
    connection.settimeout(None)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    stream = connection.makefile("rb", buffering=0)

    print(
        f"Collecting {args.max_views} diverse {args.pattern_cols}x{args.pattern_rows} "
        "inner-corner views. Keep the full board visible.",
        flush=True,
    )
    try:
        with av.open(stream, mode="r", format=args.format) as container:
            for frame in container.decode(video=0):
                frame_index += 1
                elapsed = time.monotonic() - started_at
                if elapsed >= args.duration or len(views) >= args.max_views:
                    break
                if frame_index % 3 != 0:
                    continue

                image = frame.to_ndarray(format="bgr24")
                height, width = image.shape[:2]
                image_size = (width, height)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
                accepted = False
                if found:
                    detection_count += 1
                    blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                    feature, area_ratio = view_feature(
                        corners, width, height, args.pattern_cols, args.pattern_rows
                    )
                    if area_ratio >= 0.01 and blur_score >= 25.0:
                        distance = (
                            min(float(np.linalg.norm(feature - view.feature)) for view in views)
                            if views
                            else math.inf
                        )
                        if distance >= args.diversity_threshold:
                            view = CalibrationView(
                                frame_index, image.copy(), corners.copy(), feature, area_ratio, blur_score
                            )
                            views.append(view)
                            image_path = accepted_dir / f"view-{len(views):02d}.jpg"
                            annotated = image.copy()
                            cv2.drawChessboardCorners(annotated, pattern_size, corners, True)
                            cv2.imwrite(str(image_path), annotated)
                            accepted = True
                            print(
                                f"accepted={len(views):02d}/{args.max_views} "
                                f"area={area_ratio * 100:5.1f}% blur={blur_score:6.1f}",
                                flush=True,
                            )

                now = time.monotonic()
                if not accepted and now - last_status_at >= 1.0:
                    print(
                        f"accepted={len(views):02d}/{args.max_views} "
                        f"detections={detection_count} remaining={max(0, int(args.duration - elapsed))}s; "
                        f"{guidance(views, width, height)}",
                        flush=True,
                    )
                    last_status_at = now
    finally:
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        stream.close()
        connection.close()

    if image_size is None or len(views) < args.min_views:
        print(
            f"Calibration capture failed: collected {len(views)} views; "
            f"at least {args.min_views} are required.",
            flush=True,
        )
        return 2

    rms, camera_matrix, distortion, per_view_errors, kept = calibrate(
        views, pattern_size, image_size
    )
    retained = [views[index] for index in kept]
    all_points = np.concatenate([view.corners.reshape(-1, 2) for view in retained])
    centroids = np.array([view.corners.reshape(-1, 2).mean(axis=0) for view in retained])
    width, height = image_size
    centroid_span = np.ptp(centroids, axis=0) / np.array([width, height])
    corner_min = all_points.min(axis=0) / np.array([width, height])
    corner_max = all_points.max(axis=0) / np.array([width, height])
    errors = np.asarray(per_view_errors)

    distortion_values = np.zeros(5, dtype=np.float64)
    flat_distortion = distortion.reshape(-1)
    distortion_values[: min(5, len(flat_distortion))] = flat_distortion[:5]
    calibration_values = np.concatenate(
        [
            np.array(
                [
                    camera_matrix[0, 0],
                    camera_matrix[1, 1],
                    camera_matrix[0, 2],
                    camera_matrix[1, 2],
                ]
            ),
            distortion_values,
        ]
    )

    coverage = np.full((height, width, 3), 245, dtype=np.uint8)
    for index, view in enumerate(retained):
        color = tuple(int(value) for value in cv2.applyColorMap(
            np.array([[round(255 * index / max(1, len(retained) - 1))]], dtype=np.uint8),
            cv2.COLORMAP_TURBO,
        )[0, 0])
        hull = cv2.convexHull(view.corners.astype(np.float32)).astype(np.int32)
        cv2.polylines(coverage, [hull], True, color, 2)
        center = tuple(np.round(view.corners.reshape(-1, 2).mean(axis=0)).astype(int))
        cv2.circle(coverage, center, 5, color, -1)
    coverage_path = run_dir / f"{args.name}.coverage.png"
    cv2.imwrite(str(coverage_path), coverage)

    sample = retained[len(retained) // 2].image
    undistorted = cv2.undistort(sample, camera_matrix, distortion)
    check_path = run_dir / f"{args.name}.undistortion-check.jpg"
    cv2.imwrite(str(check_path), np.hstack([sample, undistorted]))

    calibration_path = run_dir / f"{args.name}.txt"
    calibration_path.write_text(
        " ".join(f"{value:.9f}" for value in calibration_values) + "\n",
        encoding="utf-8",
    )

    quality_checks = {
        "rms_reprojection_px_at_most_1": bool(rms <= 1.0),
        "max_view_error_px_at_most_1_5": bool(errors.max() <= 1.5),
        "horizontal_centroid_span_at_least_0_35": bool(centroid_span[0] >= 0.35),
        "vertical_centroid_span_at_least_0_25": bool(centroid_span[1] >= 0.25),
        "principal_point_plausible": bool(
            0.35 * width <= camera_matrix[0, 2] <= 0.65 * width
            and 0.35 * height <= camera_matrix[1, 2] <= 0.65 * height
        ),
    }
    quality_passed = all(quality_checks.values())
    metrics = {
        "name": args.name,
        "source": {"host": args.host, "port": args.port, "format": args.format},
        "resolution": {"width": width, "height": height},
        "target": {
            "inner_corners": {"columns": args.pattern_cols, "rows": args.pattern_rows},
            "square_size_units": 1.0,
            "note": "Square size does not affect intrinsic parameters.",
        },
        "capture": {
            "decoded_frames_considered": frame_index,
            "checkerboard_detections": detection_count,
            "accepted_views": len(views),
            "retained_views": len(retained),
            "rejected_outlier_views": len(views) - len(retained),
        },
        "calibration": {
            "camera_matrix": camera_matrix.tolist(),
            "distortion_k1_k2_p1_p2_k3": distortion_values.tolist(),
            "dpvo_values": calibration_values.tolist(),
            "rms_reprojection_error_px": float(rms),
            "per_view_error_px": {
                "mean": float(errors.mean()),
                "median": float(np.median(errors)),
                "max": float(errors.max()),
            },
        },
        "coverage": {
            "centroid_span_normalized": centroid_span.tolist(),
            "corner_min_normalized": corner_min.tolist(),
            "corner_max_normalized": corner_max.tolist(),
            "board_area_ratio_min": float(min(view.area_ratio for view in retained)),
            "board_area_ratio_max": float(max(view.area_ratio for view in retained)),
        },
        "quality": {"passed": quality_passed, "checks": quality_checks},
        "artifacts": {
            "calibration": str(calibration_path),
            "coverage": str(coverage_path),
            "undistortion_check": str(check_path),
            "accepted_views": str(accepted_dir),
        },
    }
    metrics_path = run_dir / f"{args.name}.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    print(
        f"RMS={rms:.4f}px mean={errors.mean():.4f}px max={errors.max():.4f}px "
        f"fx={camera_matrix[0, 0]:.3f} fy={camera_matrix[1, 1]:.3f} "
        f"cx={camera_matrix[0, 2]:.3f} cy={camera_matrix[1, 2]:.3f}",
        flush=True,
    )
    print(f"CALIBRATION_QUALITY={'PASS' if quality_passed else 'REVIEW'}", flush=True)
    print(f"Calibration candidate: {calibration_path}", flush=True)
    print(f"Metrics: {metrics_path}", flush=True)
    return 0 if quality_passed else 3


if __name__ == "__main__":
    raise SystemExit(main())
