#!/usr/bin/env python3
"""Generate the Gate6 AMCL map with the measured lower-right fence opening."""

from pathlib import Path


WIDTH = 120
HEIGHT = 120
RESOLUTION_M = 0.05
HEADER = b"P5\n120 120\n255\n"

# The opening endpoints align with the right and bottom faces of box 3.
BOTTOM_FENCE_END_X_M = 4.70
RIGHT_FENCE_START_Y_M = 1.65

# The old map represented part of the upper boundary as a 2.05 x 0.55 m
# filled block.  The measured arena has no fourth interior obstacle, so v2
# replaces that legacy block with the one-cell physical upper fence below.
LEGACY_TOP_BLOCK_X = range(60, 101)
LEGACY_TOP_BLOCK_Y = range(0, 11)


def set_occupied(pixels, image_x, image_y):
    pixels[image_y * WIDTH + image_x] = 0


def generate(source, destination):
    data = source.read_bytes()
    if not data.startswith(HEADER):
        raise RuntimeError("expected a binary 120x120 PGM with the standard header")
    pixels = bytearray(data[len(HEADER) :])
    if len(pixels) != WIDTH * HEIGHT:
        raise RuntimeError("unexpected source map size: {} bytes".format(len(pixels)))

    for image_y in LEGACY_TOP_BLOCK_Y:
        for image_x in LEGACY_TOP_BLOCK_X:
            pixels[image_y * WIDTH + image_x] = 255

    # Full physical top and left boundaries.
    for image_x in range(WIDTH):
        set_occupied(pixels, image_x, 0)
    for image_y in range(HEIGHT):
        set_occupied(pixels, 0, image_y)

    # Bottom boundary ends at x=4.70 m; [4.70, 6.00] remains open.
    bottom_end_cell = int(round(BOTTOM_FENCE_END_X_M / RESOLUTION_M))
    for image_x in range(bottom_end_cell):
        set_occupied(pixels, image_x, HEIGHT - 1)

    # Right boundary starts at y=1.65 m; [0.00, 1.65) remains open.
    right_start_cell = int(round(RIGHT_FENCE_START_Y_M / RESOLUTION_M))
    last_occupied_image_row = HEIGHT - 1 - right_start_cell
    for image_y in range(last_occupied_image_row + 1):
        set_occupied(pixels, WIDTH - 1, image_y)

    destination.write_bytes(HEADER + pixels)


def main():
    map_dir = Path(__file__).resolve().parents[1] / "map"
    source = map_dir / "arena_amcl_manual_1.pgm"
    destination = map_dir / "arena_amcl_fence_v2.pgm"
    generate(source, destination)
    print("generated {}".format(destination))


if __name__ == "__main__":
    main()
