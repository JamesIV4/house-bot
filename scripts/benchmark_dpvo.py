#!/usr/bin/env python3
"""Headless, synchronized DPVO/DPV-SLAM benchmark for a video file."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import resource
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dpvo-root", type=Path, required=True)
    parser.add_argument("--network", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--calib", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--skip", type=int, default=0)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--loop-closure", action="store_true")
    return parser.parse_args()


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def latency_summary(values_ms: list[float], deadline_ms: float) -> dict[str, float | int]:
    deadline_misses = int(np.count_nonzero(np.asarray(values_ms) > deadline_ms))
    return {
        "mean_ms": float(np.mean(values_ms)),
        "median_ms": percentile(values_ms, 50),
        "p95_ms": percentile(values_ms, 95),
        "p99_ms": percentile(values_ms, 99),
        "max_ms": float(np.max(values_ms)),
        "deadline_ms": deadline_ms,
        "deadline_miss_count": deadline_misses,
        "deadline_miss_fraction": deadline_misses / len(values_ms),
    }


def trajectory_summary(poses: np.ndarray) -> dict[str, object]:
    positions = poses[:, :3]
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    path_length = float(steps.sum())
    displacement = float(np.linalg.norm(positions[-1] - positions[0]))
    return {
        "pose_count": int(len(poses)),
        "path_length_scale_ambiguous": path_length,
        "start_end_displacement_scale_ambiguous": displacement,
        "start_end_over_path": displacement / path_length if path_length > 0 else None,
        "bounds_min_xyz": positions.min(axis=0).tolist(),
        "bounds_max_xyz": positions.max(axis=0).tolist(),
        "bounds_extent_xyz": np.ptp(positions, axis=0).tolist(),
    }


def edge_summary(slam) -> dict[str, int]:
    pg = slam.pg
    ii = torch.cat((pg.ii_inac, pg.ii)).detach().cpu().numpy()
    jj = torch.cat((pg.jj_inac, pg.jj)).detach().cpu().numpy()
    if len(ii):
        pairs = np.stack((ii, jj), axis=1)
        unique_pairs = np.unique(pairs, axis=0)
        long_pairs = unique_pairs[(unique_pairs[:, 1] - unique_pairs[:, 0]) > 30]
    else:
        unique_pairs = np.empty((0, 2), dtype=np.int64)
        long_pairs = unique_pairs
    return {
        "active_patch_factors": int(pg.ii.numel()),
        "inactive_patch_factors": int(pg.ii_inac.numel()),
        "unique_frame_pairs": int(len(unique_pairs)),
        "long_range_frame_pairs_gap_gt_30": int(len(long_pairs)),
        "global_ba_frame_count": int(np.count_nonzero(slam.ran_global_ba)),
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_head(path: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()


def write_tum(path: Path, poses: np.ndarray, sample_timestamps: np.ndarray,
              source_fps: float, stride: int, skip: int) -> None:
    source_indices = skip + stride - 1 + sample_timestamps * stride
    seconds = source_indices / source_fps
    with path.open("w", encoding="utf-8") as handle:
        for timestamp, pose in zip(seconds, poses):
            tx, ty, tz, qx, qy, qz, qw = pose
            handle.write(
                f"{timestamp:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )


def main() -> int:
    args = parse_args()
    if args.stride < 1 or args.skip < 0:
        raise ValueError("stride must be >= 1 and skip must be >= 0")
    for path in (args.dpvo_root, args.network, args.video, args.calib, args.config):
        if not path.exists():
            raise FileNotFoundError(path)

    sys.path.insert(0, str(args.dpvo_root))
    from dpvo.config import cfg
    from dpvo.dpvo import DPVO

    cfg.merge_from_file(str(args.config))
    cfg.LOOP_CLOSURE = args.loop_closure
    cfg.CLASSIC_LOOP_CLOSURE = False

    np.random.seed(0)
    torch.manual_seed(0)
    torch.cuda.manual_seed_all(0)
    torch.backends.cudnn.benchmark = False
    torch.cuda.empty_cache()

    calibration = np.loadtxt(args.calib, delimiter=" ")
    fx, fy, cx, cy = calibration[:4]
    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
    )

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open {args.video}")
    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    source_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    source_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    source_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    source_index = -1
    for _ in range(args.skip):
        ok, _ = capture.read()
        source_index += 1
        if not ok:
            raise RuntimeError("Video ended during skip")

    slam = None
    model_load_s = 0.0
    tracking_wall_s = 0.0
    ingest_latencies_ms: list[float] = []
    gpu_latencies_ms: list[float] = []
    tracked_source_indices: list[int] = []
    retained_keyframes: list[int] = []
    global_ba_at_state: list[bool] = []
    start_event = torch.cuda.Event(enable_timing=True)
    end_event = torch.cuda.Event(enable_timing=True)

    try:
        with torch.inference_mode():
            while True:
                sample_start = time.perf_counter()
                image = None
                ok = False
                for _ in range(args.stride):
                    ok, image = capture.read()
                    source_index += 1
                    if not ok:
                        break
                if not ok:
                    break

                if len(calibration) > 4:
                    image = cv2.undistort(image, camera_matrix, calibration[4:])
                image = cv2.resize(
                    image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA
                )
                height, width, _ = image.shape
                image = image[: height - height % 16, : width - width % 16]
                intrinsics = np.array(
                    [fx * 0.5, fy * 0.5, cx * 0.5, cy * 0.5], dtype=np.float32
                )

                tensor_image = torch.from_numpy(image).permute(2, 0, 1).cuda()
                tensor_intrinsics = torch.from_numpy(intrinsics).cuda()

                if slam is None:
                    load_start = time.perf_counter()
                    slam = DPVO(
                        cfg, str(args.network), ht=image.shape[0], wd=image.shape[1], viz=False
                    )
                    torch.cuda.synchronize()
                    model_load_s = time.perf_counter() - load_start
                    torch.cuda.reset_peak_memory_stats()

                sample_timestamp = len(tracked_source_indices)
                start_event.record()
                slam(sample_timestamp, tensor_image, tensor_intrinsics)
                end_event.record()
                end_event.synchronize()
                gpu_latencies_ms.append(float(start_event.elapsed_time(end_event)))

                sample_wall_s = time.perf_counter() - sample_start
                if len(tracked_source_indices) == 0:
                    sample_wall_s -= model_load_s
                ingest_latencies_ms.append(sample_wall_s * 1000.0)
                tracking_wall_s += sample_wall_s
                tracked_source_indices.append(source_index)
                retained_keyframes.append(int(slam.n))
                global_ba_at_state.append(bool(slam.ran_global_ba[slam.n]))

                if len(tracked_source_indices) % 100 == 0:
                    current_fps = len(tracked_source_indices) / tracking_wall_s
                    print(
                        f"tracked={len(tracked_source_indices)} "
                        f"retained_keyframes={slam.n} synchronized_fps={current_fps:.2f}",
                        flush=True,
                    )
    finally:
        capture.release()

    if slam is None or not slam.is_initialized:
        raise RuntimeError("DPVO did not initialize on this recording")

    torch.cuda.synchronize()
    terminate_start = time.perf_counter()
    with torch.inference_mode():
        poses, sample_timestamps = slam.terminate()
    torch.cuda.synchronize()
    terminate_s = time.perf_counter() - terminate_start

    args.output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_path = args.output_dir / f"{args.name}.tum.txt"
    metrics_path = args.output_dir / f"{args.name}.json"
    latency_path = args.output_dir / f"{args.name}.latency.csv"
    write_tum(
        trajectory_path,
        poses,
        sample_timestamps,
        source_fps,
        args.stride,
        args.skip,
    )

    expected_sample_hz = source_fps / args.stride
    deadline_ms = 1000.0 / expected_sample_hz
    synchronized_fps = len(tracked_source_indices) / tracking_wall_s
    total_s = tracking_wall_s + terminate_s
    with latency_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "sample", "source_frame", "ingest_to_pose_ms", "gpu_slam_call_ms",
            "retained_keyframes", "global_ba_at_state",
        ))
        writer.writerows(zip(
            range(len(tracked_source_indices)), tracked_source_indices,
            ingest_latencies_ms, gpu_latencies_ms, retained_keyframes,
            (int(value) for value in global_ba_at_state),
        ))
    metrics = {
        "name": args.name,
        "mode": "DPV-SLAM" if args.loop_closure else "DPVO",
        "loop_closure": args.loop_closure,
        "upstream_commit": git_head(args.dpvo_root),
        "network": str(args.network),
        "network_sha256": sha256(args.network),
        "config": str(args.config),
        "resolved_config": cfg.dump(),
        "calibration": {
            "path": str(args.calib),
            "values_at_source_resolution": calibration.tolist(),
            "status": (
                "measured_with_distortion"
                if len(calibration) > 4
                else "pinhole_only_without_measured_distortion"
            ),
        },
        "video": {
            "path": str(args.video),
            "source_width": source_width,
            "source_height": source_height,
            "source_frames": source_frames,
            "source_fps": source_fps,
            "source_duration_s": source_frames / source_fps,
            "dpvo_width": source_width // 2 - (source_width // 2) % 16,
            "dpvo_height": source_height // 2 - (source_height // 2) % 16,
            "stride": args.stride,
            "skip": args.skip,
            "tracked_frames": len(tracked_source_indices),
            "first_source_frame_index": tracked_source_indices[0],
            "last_source_frame_index": tracked_source_indices[-1],
            "expected_sample_hz": expected_sample_hz,
        },
        "timing": {
            "model_load_s": model_load_s,
            "tracking_wall_s": tracking_wall_s,
            "termination_s": terminate_s,
            "tracking_plus_termination_s": total_s,
            "synchronized_tracking_fps": synchronized_fps,
            "tracking_plus_termination_fps": len(tracked_source_indices) / total_s,
            "realtime_margin": synchronized_fps / expected_sample_hz,
            "ingest_to_pose_latency": latency_summary(ingest_latencies_ms, deadline_ms),
            "steady_state_ingest_to_pose_latency_after_10_frames": latency_summary(
                ingest_latencies_ms[10:], deadline_ms
            ),
            "gpu_slam_call_latency": latency_summary(gpu_latencies_ms, deadline_ms),
        },
        "state": {
            "retained_keyframes": int(slam.n),
            "retained_patches": int(slam.m),
            **edge_summary(slam),
        },
        "trajectory": trajectory_summary(poses),
        "resources": {
            "gpu": torch.cuda.get_device_name(0),
            "compute_capability": list(torch.cuda.get_device_capability(0)),
            "torch": torch.__version__,
            "torch_cuda": torch.version.cuda,
            "peak_cuda_allocated_bytes": int(torch.cuda.max_memory_allocated()),
            "peak_cuda_reserved_bytes": int(torch.cuda.max_memory_reserved()),
            "process_max_rss_kib": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
            "platform": platform.platform(),
        },
        "artifacts": {
            "trajectory_tum": str(trajectory_path),
            "per_frame_latency_csv": str(latency_path),
        },
    }
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    print(json.dumps({
        "metrics": str(metrics_path),
        "trajectory": str(trajectory_path),
        "mode": metrics["mode"],
        "tracked_frames": len(tracked_source_indices),
        "retained_keyframes": slam.n,
        "synchronized_tracking_fps": synchronized_fps,
        "expected_sample_hz": expected_sample_hz,
        "realtime_margin": synchronized_fps / expected_sample_hz,
        "p95_ingest_to_pose_ms": metrics["timing"]["ingest_to_pose_latency"]["p95_ms"],
        "termination_s": terminate_s,
        "long_range_frame_pairs": metrics["state"]["long_range_frame_pairs_gap_gt_30"],
        "peak_cuda_reserved_gib": metrics["resources"]["peak_cuda_reserved_bytes"] / (1024 ** 3),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
