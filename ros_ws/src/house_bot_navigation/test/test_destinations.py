from pathlib import Path

import pytest

from house_bot_navigation.destinations import load_destinations, service_slug


def test_load_destinations(tmp_path: Path) -> None:
    path = tmp_path / "destinations.yaml"
    path.write_text(
        "destinations:\n  Kitchen:\n    x: 1.25\n    y: -2\n    yaw: 1.57\n",
        encoding="utf-8",
    )
    destination = load_destinations(path)["Kitchen"]
    assert destination.x == 1.25
    assert destination.y == -2.0
    assert destination.yaw == 1.57


def test_service_slug() -> None:
    assert service_slug("Front Hall") == "front_hall"


def test_duplicate_service_slugs_are_rejected(tmp_path: Path) -> None:
    path = tmp_path / "destinations.yaml"
    path.write_text(
        "destinations:\n"
        "  Front Hall: {x: 0, y: 0}\n"
        "  front-hall: {x: 1, y: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="duplicated"):
        load_destinations(path)

