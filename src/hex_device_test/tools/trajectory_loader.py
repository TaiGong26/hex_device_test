import gc
import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

DEFAULT_SEGMENT_DURATION = 2.0
_HEADER_READ_BYTES = 8192


def read_trajectory_metadata(json_path: Path) -> Dict[str, Optional[float]]:
    with open(json_path, encoding="utf-8") as f:
        header = f.read(_HEADER_READ_BYTES)

    def _match_number(field: str) -> Optional[float]:
        match = re.search(rf'"{field}"\s*:\s*([-+0-9.eE]+)', header)
        return float(match.group(1)) if match else None

    sample_interval_s = _match_number("sample_interval_s")
    sample_count = _match_number("sample_count")
    joint_count = _match_number("joint_count")

    if sample_interval_s is not None:
        return {
            "sample_interval_s": sample_interval_s,
            "sample_count": int(sample_count) if sample_count is not None else None,
            "joint_count": int(joint_count) if joint_count is not None else None,
        }

    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)
    return {
        "sample_interval_s": data.get("sample_interval_s"),
        "sample_count": data.get("sample_count"),
        "joint_count": data.get("joint_count"),
    }


def read_sample_interval(json_path: Path) -> float:
    meta = read_trajectory_metadata(json_path)
    interval = meta.get("sample_interval_s")
    if interval is None:
        raise ValueError(f"sample_interval_s not found in {json_path}")
    return float(interval)


def _read_sample_interval(json_path: Path) -> float:
    return read_sample_interval(json_path)


def _extract_waypoints(samples: list, stride: int) -> List[List[float]]:
    if stride < 1:
        raise ValueError(f"stride must be >= 1, got {stride}")
    waypoints = [sample["positions_rad"] for sample in samples[::stride]]
    if not waypoints:
        raise ValueError("No waypoints found in samples")
    return waypoints


def estimate_waypoints_memory_mb(waypoint_count: int, joint_count: int = 6) -> float:
    """Rough retained memory for Python list-of-lists waypoints."""
    bytes_per_waypoint = joint_count * 28 + 64
    return waypoint_count * bytes_per_waypoint / (1024 * 1024)


def load_waypoints_from_json(json_path: Path, stride: int = 1) -> List[List[float]]:
    waypoints, _ = _load_single_json(json_path, stride)
    return waypoints


def _load_single_json(json_path: Path, stride: int) -> Tuple[List[List[float]], int]:
    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)
    samples = data["samples"]
    sample_count = len(samples)
    waypoints = _extract_waypoints(samples, stride)
    del data, samples
    return waypoints, sample_count


def load_waypoints_from_json_files(
    json_paths: Sequence[Path],
    stride: int = 1,
) -> List[List[float]]:
    waypoints, _ = load_waypoints_with_segment_boundaries(json_paths, stride=stride)
    return waypoints


def load_waypoints_with_segment_boundaries(
    json_paths: Sequence[Path],
    stride: int = 1,
) -> Tuple[List[List[float]], List[int]]:
    """Load waypoints and exclusive end indices marking each JSON file's segment."""
    if not json_paths:
        raise ValueError("json_paths must not be empty")

    merged: List[List[float]] = []
    segment_ends: List[int] = []
    for json_path in json_paths:
        waypoints, _ = _load_single_json(json_path, stride)
        merged.extend(waypoints)
        segment_ends.append(len(merged))
        gc.collect()

    return merged, segment_ends


def get_replay_segment_duration(
    json_paths: Union[Path, Sequence[Path]],
    stride: int = 1,
) -> float:
    paths = [json_paths] if isinstance(json_paths, Path) else list(json_paths)
    if not paths:
        raise ValueError("json_paths must not be empty")
    if stride < 1:
        raise ValueError(f"stride must be >= 1, got {stride}")

    intervals = [read_sample_interval(path) for path in paths]
    base_interval = intervals[0]
    for path, interval in zip(paths[1:], intervals[1:]):
        if interval != base_interval:
            raise ValueError(
                f"sample_interval_s mismatch: {paths[0]} has {base_interval}, "
                f"{path} has {interval}"
            )
    return base_interval * stride
