"""Validation and naming helpers for map-frame destinations."""

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any

import yaml


@dataclass(frozen=True)
class Destination:
    name: str
    x: float
    y: float
    yaw: float
    description: str = ""


def service_slug(name: str) -> str:
    """Return a ROS-name-safe, stable service component."""
    slug = re.sub(r"[^a-z0-9_]+", "_", name.strip().lower()).strip("_")
    if not slug:
        raise ValueError(f"Destination name {name!r} has no usable characters")
    return slug


def _finite_number(value: Any, field: str, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"Destination {name!r} field {field!r} must be a number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"Destination {name!r} field {field!r} must be a number"
        ) from exc
    if not math.isfinite(number):
        raise ValueError(f"Destination {name!r} field {field!r} must be finite")
    return number


def load_destinations(path: str | Path) -> dict[str, Destination]:
    """Load and validate the project's destinations YAML file."""
    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}

    raw_destinations = document.get("destinations")
    if not isinstance(raw_destinations, dict) or not raw_destinations:
        raise ValueError(f"{source} must contain a non-empty destinations mapping")

    destinations: dict[str, Destination] = {}
    slugs: set[str] = set()
    for raw_name, raw_pose in raw_destinations.items():
        name = str(raw_name).strip()
        if not name:
            raise ValueError("Destination names cannot be empty")
        if not isinstance(raw_pose, dict):
            raise ValueError(f"Destination {name!r} must be a mapping")

        slug = service_slug(name)
        if slug in slugs:
            raise ValueError(f"Destination service name {slug!r} is duplicated")
        slugs.add(slug)

        destinations[name] = Destination(
            name=name,
            x=_finite_number(raw_pose.get("x"), "x", name),
            y=_finite_number(raw_pose.get("y"), "y", name),
            yaw=_finite_number(raw_pose.get("yaw", 0.0), "yaw", name),
            description=str(raw_pose.get("description", "")).strip(),
        )

    return destinations

