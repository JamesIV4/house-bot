from house_bot_navigation.mock_map import HEIGHT, WIDTH, ensure_mock_map


def test_mock_map_is_generated(tmp_path) -> None:
    yaml_path = ensure_mock_map(tmp_path)
    image_path = tmp_path / "mock_house.pgm"
    assert yaml_path.exists()
    assert image_path.exists()
    assert image_path.read_bytes().startswith(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode())
    assert "resolution: 0.05" in yaml_path.read_text(encoding="utf-8")

