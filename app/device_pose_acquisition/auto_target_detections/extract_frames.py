import os
import argparse
import cv2

# Default settings
DEFAULT_VIDEO_FILE = "IMG_8362.MOV"  # Detected in current folder
DEFAULT_OUTPUT_DIR = "frames_IMG_8362"
DEFAULT_TARGET_FPS = 2.0  # frames per second
DEFAULT_JPEG_QUALITY = 95  # 0-100 (higher is better)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract JPG frames from a video.")
    parser.add_argument(
        "-i",
        "--input",
        default=DEFAULT_VIDEO_FILE,
        help="Input video file (default: %(default)s)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=DEFAULT_OUTPUT_DIR,
        help="Output folder for frames (default: %(default)s)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=DEFAULT_TARGET_FPS,
        help="Target frame rate for extraction (default: %(default)s)",
    )
    parser.add_argument(
        "--quality",
        type=int,
        default=DEFAULT_JPEG_QUALITY,
        help="JPEG quality 0-100 (default: %(default)s)",
    )
    return parser.parse_args()


def main(args: argparse.Namespace | None = None):
    if args is None:
        args = parse_args()

    video_file = args.input
    output_dir = args.output
    target_fps = args.fps
    jpeg_quality = args.quality

    if not os.path.exists(video_file):
        raise FileNotFoundError(f"Video not found: {video_file}")

    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_file)
    if not cap.isOpened():
        raise RuntimeError("Failed to open video. Your OpenCV build may lack codec support.")

    src_fps = cap.get(cv2.CAP_PROP_FPS)
    if not src_fps or src_fps <= 0:
        # Fallback if FPS is unknown
        src_fps = 30.0

    frame_idx = 0
    saved = 0
    next_output_time = 0.0  # seconds
    step = 1.0 / target_fps

    imwrite_params = [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        t = frame_idx / src_fps  # current timestamp in seconds (approx)

        if t + 1e-9 >= next_output_time:
            saved += 1
            out_path = os.path.join(output_dir, f"img_{saved:06d}.jpg")
            ok = cv2.imwrite(out_path, frame, imwrite_params)
            if not ok:
                raise RuntimeError(f"Failed to write frame to {out_path}")
            next_output_time += step

    cap.release()

    # Simple report
    if saved == 0:
        print("No frames were saved. Check the video and codec support.")
    else:
        print(f"Saved {saved} frames to {output_dir} at ~{target_fps} fps.")


if __name__ == "__main__":
    main()
