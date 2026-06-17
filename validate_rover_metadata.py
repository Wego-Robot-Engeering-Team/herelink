#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable


TOP_LEVEL_REQUIRED = {
    "metaVersion": str,
    "appVersion": str,
    "draftInspectionUnitId": int,
    "drone": str,
    "startTime": str,
    "completeTime": str,
    "missionUuid": str,
    "bladePosMap": dict,
    "chamberPosMap": dict,
    "remark": str,
    "photos": list,
    "videos": list,
    "lidarData": list,
}

PHOTO_REQUIRED = {
    "fileDir": str,
    "filename": str,
    "photoTakenAt": str,
    "uniqueKey": str,
    "chamberPosition": int,
    "bladePosition": int,
    "timeZone": str,
    "sequenceId": int,
    "r": (int, float),
    "gimbalRoll": (int, float),
    "gimbalPitch": (int, float),
    "gimbalYaw": (int, float),
    "gpsAltitude": (int, float),
    "gpsLatitude": (int, float),
    "gpsLongitude": (int, float),
    "focalLength": (int, float),
    "fNumber": (int, float),
    "measuredDistanceToSurface": (int, float),
    "isManualShoot": bool,
    "ledPower": (int, float),
}

PHOTO_OPTIONAL = {
    "n": (int, float),
    "e": (int, float),
    "alt": (int, float),
    "bodyRoll": (int, float),
    "bodyPitch": (int, float),
    "bodyYaw": (int, float),
}

VIDEO_REQUIRED = {
    "fileDir": str,
    "filename1": str,
    "videoTakenAt": str,
    "videoLength": int,
    "uniqueKey": str,
    "chamberPosition": int,
    "bladePosition": int,
    "timeZone": str,
    "sequenceId": int,
    "r": (int, float),
    "gpsAltitude": (int, float),
    "gpsLatitude": (int, float),
    "gpsLongitude": (int, float),
    "ledPower": (int, float),
}

VIDEO_OPTIONAL = {
    "filename2": str,
    "n": (int, float),
    "e": (int, float),
    "alt": (int, float),
    "bodyRoll": (int, float),
    "bodyPitch": (int, float),
    "bodyYaw": (int, float),
}

LIDAR_REQUIRED = {
    "lidarTakenAt": str,
    "lidarLength": int,
    "uniqueKey": str,
    "chamberPosition": int,
    "bladePosition": int,
    "sequenceId": int,
    "r": (int, float),
    "resolutionVertical": int,
    "resolutionHorizontal": int,
    "timeStep": (int, float),
}

LIDAR_OPTIONAL = {
    "distanceToSurfaces": list,
}

ALLOWED_BLADE_POSITIONS = {"A", "B", "C", "1", "2", "3"}
ALLOWED_CHAMBER_POSITIONS = {"ACC", "LE", "CC", "TE", "TE2"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate Rover metadata JSON using PDF fields only."
    )
    parser.add_argument(
        "json_path",
        nargs="?",
        default="rover_metadata_example.json",
        help="Path to the metadata JSON file.",
    )
    return parser.parse_args()


def error(errors: list[str], message: str) -> None:
    errors.append(message)


def check_type(
    value: Any,
    expected: type[Any] | tuple[type[Any], ...],
    label: str,
    errors: list[str],
) -> None:
    if isinstance(expected, tuple):
        if not isinstance(value, expected):
            names = ", ".join(t.__name__ for t in expected)
            error(errors, f"{label} must be one of: {names}")
    elif not isinstance(value, expected):
        error(errors, f"{label} must be {expected.__name__}")


def require_fields(
    obj: dict[str, Any],
    required: dict[str, Any],
    label: str,
    errors: list[str],
) -> None:
    for field_name, expected_type in required.items():
        if field_name not in obj:
            error(errors, f"{label}.{field_name} is required")
            continue
        check_type(obj[field_name], expected_type, f"{label}.{field_name}", errors)


def check_optional_fields(
    obj: dict[str, Any],
    optional: dict[str, Any],
    label: str,
    errors: list[str],
) -> None:
    for field_name, expected_type in optional.items():
        if field_name in obj:
            check_type(obj[field_name], expected_type, f"{label}.{field_name}", errors)


def validate_map(
    mapping: dict[str, Any],
    expected_keys: Iterable[str],
    allowed_values: set[str],
    label: str,
    errors: list[str],
) -> None:
    expected_keys = list(expected_keys)
    if sorted(mapping.keys()) != expected_keys:
        error(errors, f"{label} keys must be exactly {expected_keys}")
    for key, value in mapping.items():
        if not isinstance(value, str):
            error(errors, f"{label}.{key} must be string")
        elif value not in allowed_values:
            error(
                errors,
                f"{label}.{key} must be one of {sorted(allowed_values)}, got: {value}",
            )


def validate_common_object(
    item: dict[str, Any],
    label: str,
    errors: list[str],
) -> None:
    if item["chamberPosition"] not in {0, 1, 2, 3, 4}:
        error(errors, f"{label}.chamberPosition must be 0..4")
    if item["bladePosition"] not in {0, 1, 2}:
        error(errors, f"{label}.bladePosition must be 0..2")
    if item["sequenceId"] < 0:
        error(errors, f"{label}.sequenceId must be >= 0")


def validate_file(path: Path) -> list[str]:
    errors: list[str] = []
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return [f"File not found: {path}"]
    except json.JSONDecodeError as exc:
        return [f"Invalid JSON: {exc}"]

    if not isinstance(raw, dict):
        return ["Top-level JSON must be an object"]

    require_fields(raw, TOP_LEVEL_REQUIRED, "root", errors)
    if errors:
        return errors

    if raw["metaVersion"] != "1.0":
        error(errors, "root.metaVersion must be '1.0'")

    validate_map(
        raw["bladePosMap"],
        ["0", "1", "2"],
        ALLOWED_BLADE_POSITIONS,
        "root.bladePosMap",
        errors,
    )
    validate_map(
        raw["chamberPosMap"],
        ["0", "1", "2", "3", "4"],
        ALLOWED_CHAMBER_POSITIONS,
        "root.chamberPosMap",
        errors,
    )

    for index, photo in enumerate(raw["photos"]):
        label = f"root.photos[{index}]"
        if not isinstance(photo, dict):
            error(errors, f"{label} must be an object")
            continue
        require_fields(photo, PHOTO_REQUIRED, label, errors)
        check_optional_fields(photo, PHOTO_OPTIONAL, label, errors)
        if any(field not in photo for field in PHOTO_REQUIRED):
            continue
        validate_common_object(photo, label, errors)

    for index, video in enumerate(raw["videos"]):
        label = f"root.videos[{index}]"
        if not isinstance(video, dict):
            error(errors, f"{label} must be an object")
            continue
        require_fields(video, VIDEO_REQUIRED, label, errors)
        check_optional_fields(video, VIDEO_OPTIONAL, label, errors)
        if any(field not in video for field in VIDEO_REQUIRED):
            continue
        validate_common_object(video, label, errors)

    for index, lidar in enumerate(raw["lidarData"]):
        label = f"root.lidarData[{index}]"
        if not isinstance(lidar, dict):
            error(errors, f"{label} must be an object")
            continue
        require_fields(lidar, LIDAR_REQUIRED, label, errors)
        check_optional_fields(lidar, LIDAR_OPTIONAL, label, errors)
        if any(field not in lidar for field in LIDAR_REQUIRED):
            continue
        validate_common_object(lidar, label, errors)

    return errors


def main() -> int:
    args = parse_args()
    errors = validate_file(Path(args.json_path))
    if errors:
        print("Validation failed:")
        for item in errors:
            print(f"- {item}")
        return 1

    print("Validation passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
