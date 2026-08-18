"""Create the deterministic map used by the hardware-free navigation stack."""

from pathlib import Path


WIDTH = 160
HEIGHT = 100
RESOLUTION = 0.05
ORIGIN_X = -4.0
ORIGIN_Y = -2.5


def _horizontal(grid: list[bytearray], y: int, x0: int, x1: int, width: int = 2) -> None:
    for offset in range(width):
        row = HEIGHT - 1 - min(HEIGHT - 1, y + offset)
        for x in range(max(0, x0), min(WIDTH, x1 + 1)):
            grid[row][x] = 0


def _vertical(grid: list[bytearray], x: int, y0: int, y1: int, width: int = 2) -> None:
    for offset in range(width):
        column = min(WIDTH - 1, x + offset)
        for y in range(max(0, y0), min(HEIGHT, y1 + 1)):
            grid[HEIGHT - 1 - y][column] = 0


def ensure_mock_map(output_directory: str | Path) -> Path:
    """Write the mock PGM/YAML pair if needed and return the YAML path."""
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    image_path = output / "mock_house.pgm"
    yaml_path = output / "mock_house.yaml"

    grid = [bytearray([254] * WIDTH) for _ in range(HEIGHT)]

    # Outer shell.
    _horizontal(grid, 0, 0, WIDTH - 1, 3)
    _horizontal(grid, HEIGHT - 3, 0, WIDTH - 1, 3)
    _vertical(grid, 0, 0, HEIGHT - 1, 3)
    _vertical(grid, WIDTH - 3, 0, HEIGHT - 1, 3)

    # Two interior walls with wide doorways. The origin (0, 0) remains free.
    _vertical(grid, 100, 3, 34, 2)
    _vertical(grid, 100, 52, 78, 2)
    _horizontal(grid, 78, 3, 50, 2)
    _horizontal(grid, 78, 70, 100, 2)

    # A furniture island that forces the planner to choose a side.
    for y in range(18, 34):
        for x in range(55, 72):
            grid[HEIGHT - 1 - y][x] = 0

    with image_path.open("wb") as image:
        image.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        for row in grid:
            image.write(row)

    yaml_path.write_text(
        "\n".join(
            [
                f"image: {image_path}",
                "mode: trinary",
                f"resolution: {RESOLUTION}",
                f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return yaml_path

