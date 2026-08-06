#!/usr/bin/env python3

"""Generate the DejaVuSansMono LVGL fonts used by the UI.

This script recreates the 14 px and 24 px fonts in this folder.
It uses a broad ASCII range plus the degree sign so the UI text
can render without missing-glyph boxes.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


FONT_SPECS = (
    # Edit these symbol strings directly when the UI starts using new characters.
    # Keep each entry readable instead of converting them into hex ranges.
    (14, "lv_font_dejavu_sans_mono_14.c", (
        " ",
        "0123456789",
        "RPMWVAC",                                      # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "",                                             # "abcdefghijklmnopqrstuvwxyz"
        "().%",                                         # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
    (24, "lv_font_dejavu_sans_mono_24.c", (
        " ",
        "0123456789",
        "RPMWVAC",                                      # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "",                                             # "abcdefghijklmnopqrstuvwxyz"
        "().%",                                         # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
)


def find_repo_root(script_path: Path) -> Path:
    return script_path.resolve().parents[2]


def find_source_font(repo_root: Path, override: str | None) -> Path:
    if override:
        candidate = Path(override).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"Font file not found: {candidate}")
        return candidate

    candidates = (
        repo_root / ".venv" / "Lib" / "site-packages" / "matplotlib" / "mpl-data" / "fonts" / "ttf" / "DejaVuSansMono.ttf",
        Path(r"C:\\Windows\\Fonts\\DejaVuSansMono.ttf"),
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        "Could not find DejaVuSansMono.ttf. Pass --font-path explicitly."
    )


def symbols_to_ranges(symbol_groups: tuple[str, ...]) -> list[str]:
    codepoints = sorted({ord(symbol) for group in symbol_groups for symbol in group})
    if not codepoints:
        return []

    ranges: list[str] = []
    start = previous = codepoints[0]

    for codepoint in codepoints[1:]:
        if codepoint == previous + 1:
            previous = codepoint
            continue

        ranges.append(f"{start}-{previous}" if start != previous else str(start))
        start = previous = codepoint

    ranges.append(f"{start}-{previous}" if start != previous else str(start))
    return ranges


def run_font_conv(repo_root: Path, source_font: Path, size: int, output_file: Path, symbol_groups: tuple[str, ...]) -> None:
    npx = shutil.which("npx")
    if npx is None:
        raise RuntimeError("npx was not found on PATH")

    ranges = symbols_to_ranges(symbol_groups)

    cmd = [
        npx,
        "lv_font_conv",
        "--size",
        str(size),
        "--bpp",
        "4",
        "--format",
        "lvgl",
        "--lv-include",
        "lvgl.h",
        "--font",
        str(source_font),
        "--lv-font-name",
        output_file.stem,
        "-o",
        str(output_file),
    ]

    for glyph_range in ranges:
        cmd.extend(("--range", glyph_range))

    subprocess.run(cmd, cwd=repo_root, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--font-path", help="Path to DejaVuSansMono.ttf")
    args = parser.parse_args()

    script_path = Path(__file__)
    repo_root = find_repo_root(script_path)
    source_font = find_source_font(repo_root, args.font_path)
    output_dir = script_path.parent

    for size, filename, symbol_groups in FONT_SPECS:
        run_font_conv(repo_root, source_font, size, output_dir / filename, symbol_groups)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())