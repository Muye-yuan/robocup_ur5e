#!/usr/bin/env python3
"""Add constant labels to ASCII PCD files for local perception_grasp tests."""

from __future__ import annotations

import argparse
from pathlib import Path


DEFAULT_INPUTS = ("t1.pcd", "t2.pcd", "t3.pcd", "t4.pcd")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate labeled PCD test files from xyz-only ASCII PCD files."
    )
    parser.add_argument(
        "--cloud-dir",
        default="/home/muye/robocup_ur5e/weights/graspnet/test_clouds",
        help="Directory containing t1-t4 ASCII PCD files.",
    )
    return parser.parse_args()


def split_header_and_points(lines: list[str]) -> tuple[list[str], list[str]]:
    for index, line in enumerate(lines):
        if line.strip().lower().startswith("data "):
            return lines[: index + 1], lines[index + 1 :]
    raise ValueError("PCD header missing DATA line.")


def build_labeled_header(header_lines: list[str]) -> list[str]:
    updated = []
    replaced = {
        "FIELDS": False,
        "SIZE": False,
        "TYPE": False,
        "COUNT": False,
    }
    for raw_line in header_lines:
        line = raw_line.strip()
        if line.startswith("FIELDS "):
            updated.append("FIELDS x y z label\n")
            replaced["FIELDS"] = True
        elif line.startswith("SIZE "):
            updated.append("SIZE 4 4 4 4\n")
            replaced["SIZE"] = True
        elif line.startswith("TYPE "):
            updated.append("TYPE F F F U\n")
            replaced["TYPE"] = True
        elif line.startswith("COUNT "):
            updated.append("COUNT 1 1 1 1\n")
            replaced["COUNT"] = True
        else:
            updated.append(raw_line)

    missing = [key for key, done in replaced.items() if not done]
    if missing:
        raise ValueError(f"PCD header missing required keys: {', '.join(missing)}")
    return updated


def label_points(point_lines: list[str], label_value: int) -> list[str]:
    labeled = []
    for raw_line in point_lines:
        stripped = raw_line.strip()
        if not stripped:
            continue
        fields = stripped.split()
        if len(fields) < 3:
            raise ValueError(f"Invalid point line: {raw_line!r}")
        labeled.append(f"{fields[0]} {fields[1]} {fields[2]} {label_value}\n")
    return labeled


def write_labeled_copy(src_path: Path, dst_path: Path, label_value: int) -> int:
    lines = src_path.read_text(encoding="utf-8").splitlines(keepends=True)
    header_lines, point_lines = split_header_and_points(lines)
    labeled_header = build_labeled_header(header_lines)
    labeled_points = label_points(point_lines, label_value)
    dst_path.write_text("".join(labeled_header + labeled_points), encoding="utf-8")
    return len(labeled_points)


def write_merged_file(dst_path: Path, entries: list[tuple[Path, int]]) -> int:
    total_points = 0
    merged_points: list[str] = []
    header_template: list[str] | None = None

    for src_path, label_value in entries:
        lines = src_path.read_text(encoding="utf-8").splitlines(keepends=True)
        header_lines, point_lines = split_header_and_points(lines)
        if header_template is None:
            header_template = build_labeled_header(header_lines)
        labeled_points = label_points(point_lines, label_value)
        merged_points.extend(labeled_points)
        total_points += len(labeled_points)

    if header_template is None:
        raise ValueError("No input files found for merged output.")

    final_header = []
    for raw_line in header_template:
        line = raw_line.strip()
        if line.startswith("WIDTH "):
            final_header.append(f"WIDTH {total_points}\n")
        elif line.startswith("POINTS "):
            final_header.append(f"POINTS {total_points}\n")
        else:
            final_header.append(raw_line)

    dst_path.write_text("".join(final_header + merged_points), encoding="utf-8")
    return total_points


def main() -> int:
    args = parse_args()
    cloud_dir = Path(args.cloud_dir)

    inputs = []
    for index, filename in enumerate(DEFAULT_INPUTS, start=1):
        src_path = cloud_dir / filename
        if not src_path.exists():
            raise FileNotFoundError(f"Missing input PCD: {src_path}")
        inputs.append((src_path, index))

    for src_path, label_value in inputs:
        dst_path = src_path.with_name(f"{src_path.stem}_label{label_value}.pcd")
        point_count = write_labeled_copy(src_path, dst_path, label_value)
        print(f"Wrote {dst_path} with label={label_value}, points={point_count}")

    merged_path = cloud_dir / "t1234_labeled_merged.pcd"
    merged_count = write_merged_file(merged_path, inputs)
    print(f"Wrote {merged_path} with total points={merged_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
