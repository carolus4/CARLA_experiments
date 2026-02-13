"""
Encode frame_0000.png … in a folder to a GIF (e.g. data/cloudiness_frames -> data/cloudiness.gif).
Uses ffmpeg two-pass palette for small file size. Requires ffmpeg on PATH.
"""
import argparse
import os
import subprocess
import tempfile

INPUT_PATTERN = "frame_%04d.png"
FPS = 10
SCALE_WIDTH = 640  # width in px; height auto, keeps aspect ratio


def main():
    parser = argparse.ArgumentParser(
        description="Encode a folder of frame_XXXX.png images to a GIF."
    )
    parser.add_argument(
        "folder",
        help="Input folder (e.g. cloudiness_frames or data/cloudiness_frames). "
        "Output GIF is written to the same parent dir, name derived from folder (e.g. cloudiness.gif).",
    )
    args = parser.parse_args()

    input_dir = args.folder
    if not os.path.isabs(input_dir) and not input_dir.startswith("data"):
        input_dir = os.path.join("data", input_dir)
    input_dir = os.path.normpath(input_dir)

    base = os.path.basename(input_dir)
    out_name = base.replace("_frames", "") + ".gif"
    output_path = os.path.join(os.path.dirname(input_dir) or ".", out_name)

    input_path = os.path.join(input_dir, INPUT_PATTERN)
    if not os.path.isdir(input_dir):
        raise SystemExit(f"Input directory not found: {input_dir}")
    first_frame = os.path.join(input_dir, "frame_0000.png")
    if not os.path.isfile(first_frame):
        raise SystemExit(f"Frame sequence not found (e.g. {first_frame})")

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
        palette_path = f.name
    try:
        # Pass 1: generate palette from frames
        cmd_palette = [
            "ffmpeg", "-y",
            "-framerate", str(FPS),
            "-i", input_path,
            "-vf", f"fps={FPS},scale={SCALE_WIDTH}:-1:flags=lanczos,palettegen",
            palette_path,
        ]
        try:
            result = subprocess.run(cmd_palette, capture_output=True, text=True)
        except FileNotFoundError:
            raise SystemExit("ffmpeg not found on PATH")
        if result.returncode != 0:
            raise SystemExit(f"ffmpeg palettegen failed: {result.stderr or result.stdout}")
        # Pass 2: encode GIF with palette
        cmd_gif = [
            "ffmpeg", "-y",
            "-framerate", str(FPS),
            "-i", input_path,
            "-i", palette_path,
            "-lavfi", f"fps={FPS},scale={SCALE_WIDTH}:-1:flags=lanczos[x];[x][1:v]paletteuse",
            output_path,
        ]
        result = subprocess.run(cmd_gif, capture_output=True, text=True)
        if result.returncode != 0:
            raise SystemExit(f"ffmpeg paletteuse failed: {result.stderr or result.stdout}")
    finally:
        try:
            os.unlink(palette_path)
        except FileNotFoundError:
            pass

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"Wrote {output_path} ({size_mb:.1f} MB, {SCALE_WIDTH}px wide, {FPS} fps)")


if __name__ == "__main__":
    main()
