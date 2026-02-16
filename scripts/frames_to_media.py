"""
Encode frame_0000.png … in a folder to GIF or MP4 video.
Usage: python frames_to_media.py [--format gif|video] folder_name

GIF uses the first 5 seconds (50 frames at 10 fps), ffmpeg two-pass palette for small file size.
Requires ffmpeg on PATH.
"""
import argparse
import os
import subprocess
import tempfile

INPUT_PATTERN = "frame_%04d.png"
FPS = 10
SCALE_WIDTH = 640  # width in px; height auto, keeps aspect ratio
GIF_FRAME_LIMIT = 50  # first 5 seconds at 10 fps


def resolve_paths(folder: str):
    """Normalize input dir and derive output path. Returns (input_dir, output_path)."""
    input_dir = folder
    if not os.path.isabs(input_dir) and not input_dir.startswith("data"):
        input_dir = os.path.join("data", input_dir)
    input_dir = os.path.normpath(input_dir)
    base = os.path.basename(input_dir)
    return input_dir, base, os.path.dirname(input_dir) or "."


def encode_gif(input_path: str, output_path: str) -> None:
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
        palette_path = f.name
    try:
        cmd_palette = [
            "ffmpeg", "-y",
            "-framerate", str(FPS),
            "-i", input_path,
            "-vf", f"fps={FPS},scale={SCALE_WIDTH}:-1:flags=lanczos,palettegen",
            "-vframes", str(GIF_FRAME_LIMIT),
            palette_path,
        ]
        result = subprocess.run(cmd_palette, capture_output=True, text=True)
        if result.returncode != 0:
            raise SystemExit(f"ffmpeg palettegen failed: {result.stderr or result.stdout}")
        cmd_gif = [
            "ffmpeg", "-y",
            "-framerate", str(FPS),
            "-i", input_path,
            "-i", palette_path,
            "-lavfi", f"fps={FPS},scale={SCALE_WIDTH}:-1:flags=lanczos[x];[x][1:v]paletteuse",
            "-vframes", str(GIF_FRAME_LIMIT),
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


def encode_video(input_path: str, output_path: str) -> None:
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(FPS),
        "-i", input_path,
        "-vf", f"scale={SCALE_WIDTH}:-2",  # -2 for even height (yuv420p)
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        output_path,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f"ffmpeg failed: {result.stderr or result.stdout}")


def main():
    parser = argparse.ArgumentParser(
        description="Encode a folder of frame_XXXX.png images to GIF or MP4 video."
    )
    parser.add_argument(
        "--format",
        choices=["gif", "video"],
        default="gif",
        help="Output format: gif or video (MP4). Default: gif",
    )
    parser.add_argument(
        "folder",
        help="Input folder (e.g. cloudiness_frames or data/cloudiness_frames). "
        "Output is written to the same parent dir, name derived from folder.",
    )
    args = parser.parse_args()

    input_dir, base, parent_dir = resolve_paths(args.folder)
    ext = ".gif" if args.format == "gif" else ".mp4"
    out_name = base.replace("_frames", "") + ext
    output_path = os.path.join(parent_dir, out_name)

    input_path = os.path.join(input_dir, INPUT_PATTERN)
    if not os.path.isdir(input_dir):
        raise SystemExit(f"Input directory not found: {input_dir}")
    first_frame = os.path.join(input_dir, "frame_0000.png")
    if not os.path.isfile(first_frame):
        raise SystemExit(f"Frame sequence not found (e.g. {first_frame})")

    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        raise SystemExit("ffmpeg not found on PATH")

    if args.format == "gif":
        encode_gif(input_path, output_path)
    else:
        encode_video(input_path, output_path)

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"Wrote {output_path} ({size_mb:.1f} MB, {SCALE_WIDTH}px wide, {FPS} fps)")


if __name__ == "__main__":
    main()
