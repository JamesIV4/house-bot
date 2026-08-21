#!/usr/bin/env python3
"""Run DPV-SLAM against a TCP stream while consuming only the newest frame."""

from __future__ import annotations

import argparse
import json
import platform
import resource
import socket
import sys
import threading
import time
from pathlib import Path

import av
import cv2
import numpy as np
import torch

from benchmark_dpvo import (
    edge_summary,
    git_head,
    latency_summary,
    sha256,
    trajectory_summary,
)


class LatestFrameStream:
    def __init__(self, host: str, port: int, container_format: str, source_fps: float):
        self.host = host
        self.port = port
        self.container_format = container_format
        self.source_fps = source_fps
        self._condition = threading.Condition()
        self._socket = socket.create_connection((host, port), timeout=15.0)
        self._socket.settimeout(None)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._stream = self._socket.makefile("rb", buffering=0)
        self._latest = None
        self._decode_error: Exception | None = None
        self._stop = False
        self.decoded_frames = 0
        self.started_at = time.monotonic()
        self._thread = threading.Thread(target=self._decode, daemon=True)
        self._thread.start()

    def _decode(self) -> None:
        try:
            with av.open(self._stream, mode="r", format=self.container_format) as container:
                for frame in container.decode(video=0):
                    if self._stop:
                        break
                    image = frame.to_ndarray(format="bgr24")
                    arrived_at = time.monotonic()
                    with self._condition:
                        sequence = self.decoded_frames
                        self.decoded_frames += 1
                        self._latest = (
                            sequence,
                            sequence / self.source_fps,
                            arrived_at,
                            image,
                        )
                        self._condition.notify_all()
        except Exception as exc:
            if not self._stop:
                with self._condition:
                    self._decode_error = exc
                    self._condition.notify_all()

    def newest_after(self, sequence: int, timeout: float = 15.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while (
                (self._latest is None or self._latest[0] <= sequence)
                and self._decode_error is None
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Timed out waiting for a fresh camera frame")
                self._condition.wait(remaining)
            if self._decode_error is not None:
                raise RuntimeError("Camera decoder stopped") from self._decode_error
            return self._latest

    def close(self) -> None:
        self._stop = True
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._stream.close()
        self._socket.close()
        self._thread.join(timeout=2.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dpvo-root", type=Path, required=True)
    parser.add_argument("--network", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--calib", type=Path, required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--format", default="mpegts")
    parser.add_argument("--source-fps", type=float, default=30.0)
    parser.add_argument("--pose-rate-target", type=float, default=15.0)
    parser.add_argument("--max-poses", type=int, default=300)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--allow-uninitialized", action="store_true")
    return parser.parse_args()


def write_tum(path: Path, poses: np.ndarray, timestamps: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for timestamp, pose in zip(timestamps, poses):
            tx, ty, tz, qx, qy, qz, qw = pose
            handle.write(
                f"{timestamp:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )


def post_initialization_loop_summary(
    poses: np.ndarray, timestamps: np.ndarray
) -> dict[str, float | int | list[float]] | None:
    """Compare the final pose with the nearest pose in the early initialized route."""
    poses = np.asarray(poses)
    timestamps = np.asarray(timestamps)
    if len(poses) < 3:
        return None

    window_start = float(timestamps[0] + 2.5)
    window_end = float(min(timestamps[0] + 10.0, timestamps[-1]))
    candidates = np.flatnonzero(
        (timestamps >= window_start) & (timestamps <= window_end)
    )
    if not len(candidates):
        return None

    translations = poses[:, :3]
    distances = np.linalg.norm(translations[candidates] - translations[-1], axis=1)
    matched_index = int(candidates[int(np.argmin(distances))])
    translation_residual = float(np.linalg.norm(
        translations[matched_index] - translations[-1]
    ))
    segment_steps = np.linalg.norm(
        np.diff(translations[matched_index:], axis=0), axis=1
    )
    segment_path = float(segment_steps.sum())

    first_quaternion = poses[matched_index, 3:7]
    final_quaternion = poses[-1, 3:7]
    first_norm = np.linalg.norm(first_quaternion)
    final_norm = np.linalg.norm(final_quaternion)
    rotation_residual_deg = None
    if first_norm > 0 and final_norm > 0:
        quaternion_dot = abs(float(np.dot(
            first_quaternion / first_norm, final_quaternion / final_norm
        )))
        rotation_residual_deg = float(
            np.degrees(2.0 * np.arccos(np.clip(quaternion_dot, -1.0, 1.0)))
        )

    return {
        "early_search_window_s": [window_start, window_end],
        "matched_pose_index": matched_index,
        "matched_pose_time_s": float(timestamps[matched_index]),
        "translation_residual_scale_ambiguous": translation_residual,
        "rotation_residual_deg": rotation_residual_deg,
        "path_length_from_matched_pose_scale_ambiguous": segment_path,
        "translation_residual_over_path": (
            translation_residual / segment_path if segment_path > 0 else None
        ),
    }


def main() -> int:
    args = parse_args()
    for path in (args.dpvo_root, args.network, args.config, args.calib):
        if not path.exists():
            raise FileNotFoundError(path)

    sys.path.insert(0, str(args.dpvo_root))
    from dpvo.config import cfg
    from dpvo.dpvo import DPVO

    cfg.merge_from_file(str(args.config))
    cfg.LOOP_CLOSURE = True
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
    intrinsics = np.array(
        [fx * 0.5, fy * 0.5, cx * 0.5, cy * 0.5], dtype=np.float32
    )
    distortion = calibration[4:] if len(calibration) > 4 else None
    undistort_maps = None
    undistort_size = None

    args.output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = args.output_dir / f"{args.name}.json"
    trajectory_path = args.output_dir / f"{args.name}.tum.txt"
    first_frame_path = args.output_dir / f"{args.name}.first-frame.jpg"

    stream = LatestFrameStream(args.host, args.port, args.format, args.source_fps)
    slam = None
    last_sequence = -1
    processed_sequences: list[int] = []
    ingest_latencies_ms: list[float] = []
    gpu_latencies_ms: list[float] = []
    newest_queue_age_ms: list[float] = []
    decoder_to_pose_age_ms: list[float] = []
    first_processed_arrival: float | None = None
    last_processed_arrival: float | None = None
    model_load_s = 0.0
    tracking_wall_s = 0.0
    start_event = torch.cuda.Event(enable_timing=True)
    end_event = torch.cuda.Event(enable_timing=True)

    try:
        with torch.inference_mode():
            while len(processed_sequences) < args.max_poses:
                iteration_start = time.perf_counter()
                sequence, source_time, arrived_at, image = stream.newest_after(last_sequence)
                if first_processed_arrival is None:
                    first_processed_arrival = arrived_at
                last_processed_arrival = arrived_at
                newest_queue_age_ms.append((time.monotonic() - arrived_at) * 1000.0)
                last_sequence = sequence

                if not processed_sequences:
                    cv2.imwrite(str(first_frame_path), image)

                if distortion is not None:
                    source_size = (image.shape[1], image.shape[0])
                    if undistort_maps is None or undistort_size != source_size:
                        undistort_maps = cv2.initUndistortRectifyMap(
                            camera_matrix,
                            distortion,
                            None,
                            camera_matrix,
                            source_size,
                            cv2.CV_16SC2,
                        )
                        undistort_size = source_size
                    image = cv2.remap(
                        image,
                        undistort_maps[0],
                        undistort_maps[1],
                        cv2.INTER_LINEAR,
                    )
                image = cv2.resize(
                    image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA
                )
                height, width, _ = image.shape
                image = image[: height - height % 16, : width - width % 16]
                tensor_image = torch.from_numpy(image).permute(2, 0, 1).cuda()
                tensor_intrinsics = torch.from_numpy(intrinsics).cuda()

                if slam is None:
                    load_started = time.perf_counter()
                    slam = DPVO(
                        cfg,
                        str(args.network),
                        ht=image.shape[0],
                        wd=image.shape[1],
                        viz=False,
                    )
                    torch.cuda.synchronize()
                    model_load_s = time.perf_counter() - load_started
                    torch.cuda.reset_peak_memory_stats()

                start_event.record()
                slam(source_time, tensor_image, tensor_intrinsics)
                end_event.record()
                end_event.synchronize()
                gpu_latencies_ms.append(float(start_event.elapsed_time(end_event)))

                iteration_s = time.perf_counter() - iteration_start
                if not processed_sequences:
                    iteration_s -= model_load_s
                tracking_wall_s += iteration_s
                ingest_latencies_ms.append(iteration_s * 1000.0)
                decoder_to_pose_age_ms.append((time.monotonic() - arrived_at) * 1000.0)
                processed_sequences.append(sequence)

                if len(processed_sequences) % 50 == 0:
                    skipped = sequence + 1 - len(processed_sequences)
                    print(
                        f"poses={len(processed_sequences)} decoded={stream.decoded_frames} "
                        f"dropped_stale={skipped} retained_keyframes={slam.n} "
                        f"tracking_fps={len(processed_sequences) / tracking_wall_s:.2f}",
                        flush=True,
                    )
    finally:
        decoded_at_tracking_end = stream.decoded_frames
        stream.close()

    deadline_ms = 1000.0 / args.pose_rate_target
    initialized = bool(slam is not None and slam.is_initialized)
    terminate_s = 0.0
    poses = None
    timestamps = None
    if initialized:
        torch.cuda.synchronize()
        terminate_started = time.perf_counter()
        with torch.inference_mode():
            poses, timestamps = slam.terminate()
        torch.cuda.synchronize()
        terminate_s = time.perf_counter() - terminate_started
        write_tum(trajectory_path, poses, timestamps)

    source_sequence_span = processed_sequences[-1] - processed_sequences[0]
    source_time_span_s = source_sequence_span / args.source_fps
    dropped_stale = source_sequence_span + 1 - len(processed_sequences)
    trajectory_metrics = None
    if poses is not None:
        trajectory_metrics = trajectory_summary(poses)
        trajectory_metrics["post_initialization_loop"] = (
            post_initialization_loop_summary(poses, timestamps)
        )
    metrics = {
        "name": args.name,
        "mode": "live DPV-SLAM",
        "initialized": initialized,
        "upstream_commit": git_head(args.dpvo_root),
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
        "transport": {
            "host": args.host,
            "port": args.port,
            "container": args.format,
            "source_fps_assumption": args.source_fps,
            "decoded_frames_at_tracking_end": decoded_at_tracking_end,
            "processed_poses": len(processed_sequences),
            "first_source_sequence": processed_sequences[0],
            "last_source_sequence": processed_sequences[-1],
            "host_monotonic_stream_started_s": stream.started_at,
            "first_processed_arrival_monotonic_s": first_processed_arrival,
            "last_processed_arrival_monotonic_s": last_processed_arrival,
            "source_time_span_s": source_time_span_s,
            "effective_pose_hz_over_source_span": (
                (len(processed_sequences) - 1) / source_time_span_s
                if source_time_span_s > 0 else None
            ),
            "stale_frames_dropped": dropped_stale,
            "stale_drop_fraction": dropped_stale / (source_sequence_span + 1),
        },
        "timing": {
            "model_load_s": model_load_s,
            "tracking_wall_s": tracking_wall_s,
            "termination_s": terminate_s,
            "synchronized_tracking_fps": len(processed_sequences) / tracking_wall_s,
            "ingest_to_pose_latency": latency_summary(ingest_latencies_ms, deadline_ms),
            "steady_state_ingest_to_pose_latency_after_10_frames": latency_summary(
                ingest_latencies_ms[10:], deadline_ms
            ),
            "gpu_slam_call_latency": latency_summary(gpu_latencies_ms, deadline_ms),
            "newest_frame_queue_age_ms": latency_summary(newest_queue_age_ms, deadline_ms),
            "decoder_to_pose_age_ms": latency_summary(decoder_to_pose_age_ms, deadline_ms),
        },
        "state": {
            "retained_keyframes": int(slam.n) if slam is not None else 0,
            "retained_patches": int(slam.m) if slam is not None else 0,
            **(edge_summary(slam) if initialized else {}),
        },
        "trajectory": trajectory_metrics,
        "resources": {
            "gpu": torch.cuda.get_device_name(0),
            "compute_capability": list(torch.cuda.get_device_capability(0)),
            "torch": torch.__version__,
            "torch_cuda": torch.version.cuda,
            "pyav": av.__version__,
            "peak_cuda_allocated_bytes": int(torch.cuda.max_memory_allocated()),
            "peak_cuda_reserved_bytes": int(torch.cuda.max_memory_reserved()),
            "process_max_rss_kib": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
            "platform": platform.platform(),
        },
        "artifacts": {
            "metrics": str(metrics_path),
            "first_frame": str(first_frame_path),
            "trajectory_tum": str(trajectory_path) if initialized else None,
        },
    }
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    print(json.dumps({
        "metrics": str(metrics_path),
        "initialized": initialized,
        "processed_poses": len(processed_sequences),
        "decoded_frames": decoded_at_tracking_end,
        "stale_frames_dropped": dropped_stale,
        "tracking_fps": metrics["timing"]["synchronized_tracking_fps"],
        "p95_decoder_to_pose_ms": metrics["timing"]["decoder_to_pose_age_ms"]["p95_ms"],
        "retained_keyframes": metrics["state"]["retained_keyframes"],
    }, indent=2))

    if not initialized and not args.allow_uninitialized:
        print("DPV-SLAM did not initialize; move the camera during the next run.", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
