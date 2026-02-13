"""
Encode data/sunset_frames/frame_0000.png … frame_0062.png to data/sunset.gif.
Uses ffmpeg two-pass palette for small file size. Requires ffmpeg on PATH.
"""
import os
import subprocess
import tempfile

INPUT_DIR = "data/sunset_frames"
INPUT_PATTERN = "frame_%04d.png"
OUTPUT_PATH = "data/sunset.gif"
FPS = 10
SCALE_WIDTH = 640  # width in px; height auto, keeps aspect ratio


def main():
    input_path = os.path.join(INPUT_DIR, INPUT_PATTERN)
    if not os.path.isdir(INPUT_DIR):
        raise SystemExit(f"Input directory not found: {INPUT_DIR}")
    first_frame = os.path.join(INPUT_DIR, "frame_0000.png")
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
            OUTPUT_PATH,
        ]
        result = subprocess.run(cmd_gif, capture_output=True, text=True)
        if result.returncode != 0:
            raise SystemExit(f"ffmpeg paletteuse failed: {result.stderr or result.stdout}")
    finally:
        try:
            os.unlink(palette_path)
        except FileNotFoundError:
            pass

    size_mb = os.path.getsize(OUTPUT_PATH) / (1024 * 1024)
    print(f"Wrote {OUTPUT_PATH} ({size_mb:.1f} MB, {SCALE_WIDTH}px wide, {FPS} fps)")


if __name__ == "__main__":
    main()
