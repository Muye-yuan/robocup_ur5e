#!/usr/bin/env python3
"""
Pure-Python GraspNet inference core for offline and ROS reuse.
"""

from __future__ import annotations

import os
import sys
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch

try:
    import open3d as o3d
except ImportError:
    o3d = None


@dataclass
class GraspCandidateData:
    translation: np.ndarray  # shape: [3]
    rotation: np.ndarray  # shape: [3, 3]
    score: float
    width: float


def load_point_cloud(cloud_path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    path = Path(cloud_path)
    if not path.exists():
        raise FileNotFoundError(f"Point cloud file not found: {cloud_path}")
    if path.suffix.lower() not in {".ply", ".pcd"}:
        raise ValueError(f"Unsupported point cloud format: {path.suffix}. Use .ply or .pcd")

    if o3d is not None:
        pcd = o3d.io.read_point_cloud(str(path))
        points = np.asarray(pcd.points, dtype=np.float32)
        if points.size == 0:
            raise ValueError(f"Point cloud is empty: {cloud_path}")

        colors: Optional[np.ndarray] = None
        if len(pcd.colors) > 0:
            colors = np.asarray(pcd.colors, dtype=np.float32)
        return points, colors

    if path.suffix.lower() == ".ply":
        points = _load_ascii_ply_points(path)
    else:
        points = _load_ascii_pcd_points(path)
    if points.size == 0:
        raise ValueError(f"Point cloud is empty: {cloud_path}")
    colors = None

    return points, colors


def _load_ascii_ply_points(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        header = []
        for line in f:
            header.append(line.strip())
            if line.strip() == "end_header":
                break
        if not header or header[0] != "ply":
            raise ValueError(f"Not a valid ASCII PLY file: {path}")
        fmt = next((h for h in header if h.startswith("format ")), "")
        if "ascii" not in fmt:
            raise ValueError(f"Only ASCII PLY is supported without open3d: {path}")
        vline = next((h for h in header if h.startswith("element vertex ")), None)
        if vline is None:
            raise ValueError(f"PLY vertex count not found: {path}")
        n = int(vline.split()[-1])
        pts = []
        for _ in range(n):
            line = f.readline()
            if not line:
                break
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.asarray(pts, dtype=np.float32)


def _load_ascii_pcd_points(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        lines = f.readlines()
    data_idx = None
    for i, line in enumerate(lines):
        if line.strip().upper().startswith("DATA "):
            data_idx = i
            break
    if data_idx is None:
        raise ValueError(f"PCD DATA section not found: {path}")
    data_mode = lines[data_idx].strip().split()[-1].lower()
    if data_mode != "ascii":
        raise ValueError(f"Only ASCII PCD is supported without open3d: {path}")
    pts = []
    for line in lines[data_idx + 1 :]:
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.asarray(pts, dtype=np.float32)


class GraspNetInferenceCore:
    def __init__(
        self,
        checkpoint_path: str,
        device: str = "auto",
        num_point: int = 20000,
        voxel_size: float = 0.0,
        graspnet_repo_path: Optional[str] = None,
    ) -> None:
        self.repo_root = self._resolve_repo_root()
        self.checkpoint_path = self._resolve_checkpoint_path(checkpoint_path, self.repo_root)
        self.device = self._resolve_device(device)
        self.num_point = int(num_point)
        self.voxel_size = float(voxel_size)
        default_repo = str(self.repo_root / "weights" / "graspnet" / "graspnet-baseline")
        self.graspnet_repo_path = (
            graspnet_repo_path
            or os.environ.get("GRASPNET_BASELINE_DIR")
            or default_repo
        )

        self._model = None
        self._pred_decode = None

    @staticmethod
    def _resolve_repo_root() -> Path:
        # .../robocup_ur5e/src/perception_grasp/nodes/grasp_inference_core.py
        return Path(__file__).resolve().parents[3]

    @staticmethod
    def _resolve_device(device: str) -> torch.device:
        if device == "auto":
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        return torch.device(device)

    @staticmethod
    def _resolve_checkpoint_path(checkpoint_path: str, repo_root: Path) -> str:
        raw = os.path.expanduser(checkpoint_path)
        path = Path(raw)
        if not path.is_absolute():
            candidate = repo_root / path
            path = candidate if candidate.exists() else Path(os.path.abspath(raw))
        abs_path = str(path.resolve())
        if not os.path.exists(abs_path):
            raise FileNotFoundError(
                f"Checkpoint file not found: {abs_path}. "
                "Use --checkpoint_path or config.model.checkpoint_path to set it."
            )
        if os.path.getsize(abs_path) <= 0:
            raise ValueError(
                f"Checkpoint file is empty (0B): {abs_path}. "
                "Please place a valid GraspNet checkpoint file."
            )
        return abs_path

    def _ensure_graspnet_importable(self) -> None:
        if not self.graspnet_repo_path:
            raise FileNotFoundError(
                "graspnet_repo_path is not set. "
                "Expected baseline source at weights/graspnet/graspnet-baseline "
                "or set config.model.graspnet_repo_path / GRASPNET_BASELINE_DIR."
            )
        repo_path = os.path.abspath(os.path.expanduser(self.graspnet_repo_path))
        if not os.path.isdir(repo_path):
            raise FileNotFoundError(
                f"GraspNet baseline directory not found: {repo_path}. "
                "Please put baseline source at "
                "/home/muye/robocup_ur5e/weights/graspnet/graspnet-baseline "
                "or set config.model.graspnet_repo_path explicitly."
            )
        path_candidates = [
            repo_path,
            os.path.join(repo_path, "models"),
            os.path.join(repo_path, "utils"),
            os.path.join(repo_path, "pointnet2"),
            os.path.join(repo_path, "knn"),
        ]
        for p in path_candidates:
            if os.path.isdir(p) and p not in sys.path:
                sys.path.insert(0, p)

    def _build_model(self) -> None:
        if self._model is not None and self._pred_decode is not None:
            return

        self._ensure_graspnet_importable()
        try:
            from models.graspnet import GraspNet, pred_decode  # type: ignore
        except Exception as exc:
            raise ImportError(
                "Cannot import GraspNet baseline modules (expected: models.graspnet). "
                "Install graspnet-baseline and ensure its repo root is in PYTHONPATH, "
                "or set GRASPNET_BASELINE_DIR to that repo path."
            ) from exc

        model = GraspNet(
            input_feature_dim=0,
            num_view=300,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        model.to(self.device)
        model.eval()

        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        state_dict = checkpoint.get("model_state_dict", checkpoint)
        model.load_state_dict(state_dict, strict=False)

        self._model = model
        self._pred_decode = pred_decode

    def preprocess_pointcloud(
        self, points: np.ndarray, colors: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape [N, 3]")

        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        if colors is not None:
            colors = colors[valid_mask]

        if points.shape[0] == 0:
            raise ValueError("No valid points after removing NaN/Inf")

        if self.voxel_size > 0 and o3d is not None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
            if colors is not None and colors.shape[0] == points.shape[0]:
                pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0).astype(np.float64))
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            points = np.asarray(pcd.points, dtype=np.float32)
            if len(pcd.colors) > 0:
                colors = np.asarray(pcd.colors, dtype=np.float32)
            elif colors is not None:
                colors = None
        elif self.voxel_size > 0 and o3d is None:
            warnings.warn("open3d not installed; voxel downsample is skipped", RuntimeWarning)

        if points.shape[0] == 0:
            raise ValueError("No points left after preprocessing")

        if points.shape[0] >= self.num_point:
            idx = np.random.choice(points.shape[0], self.num_point, replace=False)
        else:
            idx = np.random.choice(points.shape[0], self.num_point, replace=True)

        points = points[idx].astype(np.float32)
        if colors is not None:
            colors = colors[idx].astype(np.float32)

        return points, colors

    def predict(
        self, points: np.ndarray, colors: Optional[np.ndarray] = None, top_k: int = 10
    ) -> List[Dict[str, Any]]:
        self._build_model()
        assert self._model is not None
        assert self._pred_decode is not None

        points_proc, colors_proc = self.preprocess_pointcloud(points, colors)

        end_points: Dict[str, Any] = {
            "point_clouds": torch.from_numpy(points_proc).unsqueeze(0).to(self.device)
        }
        if colors_proc is not None:
            end_points["cloud_colors"] = torch.from_numpy(colors_proc).unsqueeze(0).to(self.device)

        with torch.no_grad():
            end_points = self._model(end_points)
            grasp_preds = self._pred_decode(end_points)

        if not grasp_preds:
            return []

        first = grasp_preds[0]
        if torch.is_tensor(first):
            first = first.detach().cpu().numpy()
        first = np.asarray(first)

        if first.ndim != 2 or first.shape[1] < 16:
            raise RuntimeError(
                f"Unexpected GraspNet prediction shape: {first.shape}. "
                "Expected at least 16 columns to parse score/width/rotation/translation."
            )

        scores = first[:, 0]
        top_idx = np.argsort(scores)[::-1][: max(int(top_k), 0)]

        candidates: List[Dict[str, Any]] = []
        for idx in top_idx:
            row = first[idx]
            rotation = row[4:13].reshape(3, 3).astype(np.float32)
            translation = row[13:16].astype(np.float32)
            candidate = GraspCandidateData(
                translation=translation,
                rotation=rotation,
                score=float(row[0]),
                width=float(row[1]),
            )
            candidates.append(
                {
                    "translation": candidate.translation,
                    "rotation": candidate.rotation,
                    "score": candidate.score,
                    "width": candidate.width,
                }
            )
        return candidates
