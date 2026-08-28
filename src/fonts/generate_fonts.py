#!/usr/bin/env python3

"""Generate the LVGL fonts used by the UI.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


FONT_SPECS = (
    # Edit these symbol strings directly when the UI starts using new characters.
    # Keep each entry readable instead of converting them into hex ranges.
    # Sizes below are .rodata (flash) bytes measured with arm-none-eabi-size.
    # 10px: 551 B (digits/space/., custom font, no built-in equivalent used)
    ("Montserrat-Regular.ttf", 10, "lv_font_montserrat_10_digits.c", (
        " ",
        "0123456789",
        "",                                             # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "",                                             # "abcdefghijklmnopqrstuvwxyz"
        ".,",                                           # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "",
    )),
    # Replaces the LVGL built-in LV_FONT_MONTSERRAT_12/14/18/24 (full ASCII+Latin-1).
    # Source: Montserrat-Medium.ttf (same weight the LVGL built-ins are generated from).
    # Shared glyph set for all sizes; names need the _ui suffix because the generated
    # .c guard (LV_FONT_MONTSERRAT_<SIZE>_UI) must not collide with lv_conf.h switches.
    # 12px: built-in LV_FONT_MONTSERRAT_12 (full ASCII+Latin-1) 11,590 B -> lv_font_montserrat_12_ui 5,011 B
    ("Montserrat-Medium.ttf", 12, "lv_font_montserrat_12_ui.c", (
        " ",
        "0123456789",
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ",                   # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "abcdefghijklmnopqrstuvwxyz",                   # "abcdefghijklmnopqrstuvwxyz"
        "()#%&-./:",                                    # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
    # 14px: built-in LV_FONT_MONTSERRAT_14 (full ASCII+Latin-1) 13,751 B -> lv_font_montserrat_14_ui 5,411 B
    ("Montserrat-Medium.ttf", 14, "lv_font_montserrat_14_ui.c", (
        " ",
        "0123456789",
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ",                   # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "abcdefghijklmnopqrstuvwxyz",                   # "abcdefghijklmnopqrstuvwxyz"
        "()#%&-./:",                                    # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
    # 18px: built-in LV_FONT_MONTSERRAT_18 (full ASCII+Latin-1) 19,132 B -> lv_font_montserrat_18_ui 6,533 B
    ("Montserrat-Medium.ttf", 18, "lv_font_montserrat_18_ui.c", (
        " ",
        "0123456789",
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ",                   # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "abcdefghijklmnopqrstuvwxyz",                   # "abcdefghijklmnopqrstuvwxyz"
        "()#%&-./:",                                    # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
    # 24px: built-in LV_FONT_MONTSERRAT_24 (full ASCII+Latin-1) 28,803 B -> lv_font_montserrat_24_ui 8,378 B
    ("Montserrat-Medium.ttf", 24, "lv_font_montserrat_24_ui.c", (
        " ",
        "0123456789",
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ",                   # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "abcdefghijklmnopqrstuvwxyz",                   # "abcdefghijklmnopqrstuvwxyz"
        "()#%&-./:",                                    # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
    # 14px mono: 1,032 B
    ("DejaVuSansMono.ttf", 14, "lv_font_dejavu_sans_mono_14.c", (
        " ",
        "0123456789",
        "RPMWVAC",                                      # ABCDEFGHIJKLMNOPQRSTUVWXYZ
        "",                                             # "abcdefghijklmnopqrstuvwxyz"
        "().%",                                         # !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~
        "°",
    )),
    # 24px mono: 2,007 B
    ("DejaVuSansMono.ttf", 24, "lv_font_dejavu_sans_mono_24.c", (
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


def find_source_font(repo_root: Path, font_name: str, override: str | None) -> Path:
    if override:
        candidate = Path(override).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"Font file not found: {candidate}")
        return candidate

    candidates = (
        repo_root / ".venv" / "Lib" / "site-packages" / "matplotlib" / "mpl-data" / "fonts" / "ttf" / font_name,
        repo_root / "src" / "fonts" / font_name,
        Path(r"C:\\Windows\\Fonts\\" + font_name),
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        f"Could not find {font_name}"
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
    parser.add_argument("--font-path", help="Path to .ttf folder")
    args = parser.parse_args()

    script_path = Path(__file__)
    repo_root = find_repo_root(script_path)
    output_dir = script_path.parent

    for source_font_name, size, filename, symbol_groups in FONT_SPECS:
        source_font = find_source_font(repo_root, source_font_name, args.font_path)
        run_font_conv(repo_root, source_font, size, output_dir / filename, symbol_groups)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
