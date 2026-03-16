#!/usr/bin/env python3
"""
Offline GraspNet inference entrypoint (no ROS topic subscription).
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
NODES_DIR = PKG_DIR / "nodes"
if str(NODES_DIR) not in sys.path:
    sys.path.insert(0, str(NODES_DIR))

from grasp_inference_core import GraspNetInferenceCore, load_point_cloud  # noqa: E402


def _repo_root() -> Path:
    return PKG_DIR.parent.parent


def _resolve_path(path_text: str) -> str:
    raw = os.path.expanduser(path_text)
    path = Path(raw)
    if path.is_absolute():
        return str(path)
    candidate = _repo_root() / path
    if candidate.exists():
        return str(candidate)
    return str(Path.cwd() / path)


def _resolve_cloud_path(cloud_path: str) -> str:
    direct = Path(_resolve_path(cloud_path))
    if direct.exists():
        return str(direct)
    weights_candidate = _repo_root() / "weights" / "graspnet" / "test_clouds" / cloud_path
    if weights_candidate.exists():
        return str(weights_candidate)
    return str(direct)


def _load_yaml(path: str) -> Dict[str, Any]:
    file_path = Path(path)
    if not file_path.exists():
        return {}
    with file_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline GraspNet inference for .ply/.pcd point clouds")
    parser.add_argument("--cloud_path", required=True, help="Path to point cloud file (.ply or .pcd)")
    parser.add_argument("--checkpoint_path", default=None, help="Path to GraspNet checkpoint file")
    parser.add_argument("--top_k", type=int, default=None, help="Number of top grasp candidates to keep")
    parser.add_argument(
        "--config",
        default=str(PKG_DIR / "config" / "offline_grasp_test.yaml"),
        help="Path to offline config yaml",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    cfg = _load_yaml(args.config)

    model_cfg = cfg.get("model", {})
    infer_cfg = cfg.get("inference", {})
    preprocess_cfg = cfg.get("preprocess", {})

    checkpoint_path = args.checkpoint_path or model_cfg.get("checkpoint_path")
    if not checkpoint_path:
        raise ValueError("checkpoint_path is required (CLI --checkpoint_path or config.model.checkpoint_path)")
    checkpoint_path = _resolve_path(checkpoint_path)

    top_k = int(args.top_k if args.top_k is not None else infer_cfg.get("top_k", 10))
    device = infer_cfg.get("device", "auto")
    num_point = int(preprocess_cfg.get("num_point", 20000))
    voxel_size = float(preprocess_cfg.get("voxel_size", 0.0))
    graspnet_repo_path = model_cfg.get("graspnet_repo_path", os.environ.get("GRASPNET_BASELINE_DIR"))
    if graspnet_repo_path:
        graspnet_repo_path = _resolve_path(graspnet_repo_path)

    cloud_path = _resolve_cloud_path(args.cloud_path)
    points, colors = load_point_cloud(cloud_path)
    print(f"[Offline] Loaded cloud: {cloud_path}")
    print(f"[Offline] Raw points: {points.shape[0]}")
    print(f"[Offline] Has colors: {'yes' if colors is not None else 'no'}")
    print(f"[Offline] Checkpoint: {checkpoint_path}")
    print(f"[Offline] Baseline repo: {graspnet_repo_path or '(default)'}")

    core = GraspNetInferenceCore(
        checkpoint_path=checkpoint_path,
        device=device,
        num_point=num_point,
        voxel_size=voxel_size,
        graspnet_repo_path=graspnet_repo_path,
    )
    points_proc, colors_proc = core.preprocess_pointcloud(points, colors)
    print(f"[Offline] Preprocessed points: {points_proc.shape[0]}")

    start = time.perf_counter()
    candidates = core.predict(points, colors, top_k=top_k)
    elapsed = time.perf_counter() - start

    print(f"[Offline] Inference time: {elapsed:.4f}s")
    print(f"[Offline] Candidate count: {len(candidates)}")

    if candidates:
        top1 = candidates[0]
        translation = np.asarray(top1["translation"]).reshape(3).tolist()
        rotation = np.asarray(top1["rotation"]).reshape(3, 3).tolist()
        print("[Offline] Top-1 grasp:")
        print(f"  translation: {translation}")
        print(f"  rotation: {rotation}")
        print(f"  score: {float(top1['score']):.6f}")
        print(f"  width: {float(top1['width']):.6f}")
    else:
        print("[Offline] No grasp candidates returned.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
