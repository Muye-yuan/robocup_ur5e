# Perception Grasp Offline (Stage 1)

This is the minimal offline GraspNet workflow without ROS topic subscription.

## Scope

- Read local point cloud file (`.ply` or `.pcd`)
- Run real GraspNet inference (no fake grasp fallback)
- Print top-k grasp candidates in terminal

## Files

- Core: `src/perception_grasp/nodes/grasp_inference_core.py`
- CLI: `src/perception_grasp/tools/offline_graspnet_infer.py`
- Config: `src/perception_grasp/config/offline_grasp_test.yaml`

## Prerequisites

1. Install Python deps:
```bash
pip install -r src/perception_grasp/requirements.txt
```

2. Prepare model assets strictly under `weights/graspnet/`:
```text
weights/graspnet/
├── checkpoint.tar
├── graspnet-baseline/
└── test_clouds/
```

- `checkpoint.tar`: real checkpoint file (must not be 0B)
- `graspnet-baseline/`: baseline repo root (must contain `models/graspnet.py`)
- `test_clouds/`: your `.ply` / `.pcd` samples

If baseline path differs, set one of:
- `model.graspnet_repo_path` in `config/offline_grasp_test.yaml`
- `GRASPNET_BASELINE_DIR` environment variable

## Run

```bash
python3 src/perception_grasp/tools/offline_graspnet_infer.py \
  --cloud_path weights/graspnet/test_clouds/sample.ply \
  --checkpoint_path weights/graspnet/checkpoint.tar \
  --top_k 10
```

Or use config defaults:
```bash
python3 src/perception_grasp/tools/offline_graspnet_infer.py \
  --cloud_path sample.pcd
```

## Output

The script prints:
- loaded point count
- preprocessed point count
- inference time
- candidate count
- top-1 translation / rotation / score / width

## Failure Diagnostics

- Baseline missing: clear error with expected path
  `/home/muye/robocup_ur5e/weights/graspnet/graspnet-baseline`
- Checkpoint missing: clear `FileNotFoundError`
- Checkpoint is 0B: clear `ValueError`
