# Perception Grasp Runbook (Offline GraspNet)

This runbook records the minimum reproducible steps for offline GraspNet inference (no ROS topics).

## 1) Required Files

Use these exact paths:

- `weights/graspnet/graspnet-baseline`
- `weights/graspnet/checkpoint.tar` (valid, non-empty)
- `weights/graspnet/test_clouds/min_cube.ply` (or your own `.ply` / `.pcd`)

Do not use:

- `weights/graspnet/checkpoint-rs.tar` if it is `0B`.

## 2) Enter Container

Run on host:

```bash
cd /home/muye/robocup_ur5e
docker exec -it perception_grasp bash
```

Inside container:

```bash
cd /workspace
pip3 install -r src/perception_grasp/requirements.txt
```

## 3) Build Required Extensions

### 3.1 pointnet2

```bash
cd /workspace/weights/graspnet/graspnet-baseline/pointnet2
mkdir -p pointnet2
python3 setup.py clean --all
python3 setup.py build_ext --inplace
find /workspace/weights/graspnet/graspnet-baseline/pointnet2 -maxdepth 3 -name "*_ext*.so" -o -name "_ext*.so"
```

Expected artifact example:

- `pointnet2/pointnet2/_ext.cpython-38-x86_64-linux-gnu.so`

### 3.2 knn

```bash
cd /workspace/weights/graspnet/graspnet-baseline/knn
mkdir -p knn_pytorch
python3 setup.py build_ext --inplace
ls -lh /workspace/weights/graspnet/graspnet-baseline/knn/knn_pytorch
```

Expected artifact example:

- `knn_pytorch/knn_pytorch.cpython-38-x86_64-linux-gnu.so`

## 4) Verify Extension Imports

```bash
python3 - <<'PY'
import sys
sys.path.insert(0, '/workspace/weights/graspnet/graspnet-baseline/pointnet2')
sys.path.insert(0, '/workspace/weights/graspnet/graspnet-baseline/knn')
import pointnet2._ext
import knn_pytorch.knn_pytorch
print('OK: pointnet2._ext + knn_pytorch')
PY
```

## 5) Run Offline Inference

```bash
cd /workspace
python3 src/perception_grasp/tools/offline_graspnet_infer.py \
  --cloud_path weights/graspnet/test_clouds/min_cube.ply \
  --checkpoint_path weights/graspnet/checkpoint.tar \
  --top_k 5
```

Success output should contain:

- `[Offline] Loaded cloud: ...`
- `[Offline] Preprocessed points: ...`
- `[Offline] Inference time: ...`
- `[Offline] Candidate count: ...`
- `[Offline] Top-1 grasp: ...`

## 6) Quick Failure Triage

- `No module named pointnet2._ext`:
  - Rebuild `pointnet2` and verify `.so` exists.
- `No module named knn_pytorch`:
  - Rebuild `knn` and verify `.so` exists.
- `ImportError: libc10.so`:
  - Add PyTorch lib path to `LD_LIBRARY_PATH`.
- `Checkpoint file is empty (0B)`:
  - Replace with valid `weights/graspnet/checkpoint.tar`.

## 7) One-Line Recheck Before New Work

```bash
python3 src/perception_grasp/tools/offline_graspnet_infer.py --cloud_path weights/graspnet/test_clouds/min_cube.ply --checkpoint_path weights/graspnet/checkpoint.tar --top_k 3
```

