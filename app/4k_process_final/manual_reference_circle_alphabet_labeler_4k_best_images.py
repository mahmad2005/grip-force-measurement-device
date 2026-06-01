"""
Manual-reference circle alphabet labeling for your board.

How it works:
1) Opens the selected reference frame.
2) Asks you to click the center of A, B, C, D, E, F, G, H, I, J, 0 one by one.
3) Saves those clicked reference locations into JSON.
4) Detects circle centers in each frame.
5) Assigns each detected circle to the nearest manually clicked reference label.
6) Saves annotated video + coordinate TXT.

IMPORTANT:
This script needs normal OpenCV GUI support for the first-time clicking window.
If cv2.imshow gives an error, run:
    pip uninstall opencv-python-headless
    pip install opencv-python pupil-apriltags numpy
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except Exception as exc:
    raise SystemExit(
        "Missing dependency 'pupil-apriltags'. Install with:\n"
        "    pip install pupil-apriltags opencv-python numpy"
    ) from exc


# ============================================================
# EDIT ONLY THIS TOP SECTION
# ============================================================

#VIDEO_FILE = "./4kto2k_converter/converted_videos/IMG_7371_fixed_1080p30.mp4"
VIDEO_FILE = "IMG_8363.MOV"

OUTPUT_DIR = "./manual_alphabet_output_63_2"

# ============================================================
# 4K / 2K AUTO-SCALING SETTINGS
# ============================================================
# Your old tuning was for 1920x1080 video. When the input is 3840x2160,
# the code will automatically multiply contour sizes, kernel sizes,
# line thickness, match distance, and text size by about 2x.
BASE_TUNING_WIDTH = 1920
BASE_TUNING_HEIGHT = 1080
AUTO_SCALE_TO_VIDEO_RESOLUTION = True

# Camera calibration resolution. Keep this equal to the resolution used
# when the camera matrix below was created. If your calibration was done
# from 1920x1080 frames, leave this as 1920x1080.
CALIBRATION_FRAME_WIDTH = 1080
CALIBRATION_FRAME_HEIGHT = 1920

# For 4K output, labels/text can become too small if left unchanged.
AUTO_SCALE_DRAWING = True

# Phone videos may be stored with portrait/rotation metadata.
# OpenCV often ignores that metadata, so enable this when the video appears sideways/cut.
# Options: None, "clockwise", "counterclockwise", "180"
ROTATE_FRAME = None

# Use 0 to process full video. Use 30 for first 30 frames.
MAX_FRAMES = 10

# Frame used only for manual clicking/reference selection.
# Example:
#   0  = first frame
#   30 = around 1 second if video is 30 FPS
#   60 = around 2 seconds if video is 30 FPS
# This does NOT change processing start frame. Processing still starts from frame 0.
REFERENCE_FRAME_INDEX = 1

# If True, it will ignore old saved reference points and ask you to click again.
FORCE_RECLICK_REFERENCE = False

# Reference file will be saved here.
REFERENCE_JSON = "manual_reference_points.json"

# Labels to click one by one.
# Change order if you want.
REFERENCE_LABELS = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "0"]

# Match threshold.
# If using Tag-1 coordinates, this unit depends on your tag_size_m setting.
# If too many unknown labels: increase this value.
# If wrong labels: decrease this value.
MAX_MATCH_DISTANCE_TAG = 10.0

# Pixel fallback threshold if Tag-1 pose is not available.
MAX_MATCH_DISTANCE_PX = 90.0

# Use AprilTag-1 plane coordinates for matching.
# This is more robust than raw pixel matching if camera moves slightly.
USE_TAG1_COORDINATES = True

REF_TAG_ID = 1
APRILTAG_FAMILY = "tag16h5"

# Keep same value as your existing code.
TAG_SIZE_M = 7.3

MIN_DECISION_MARGIN = 10.0
MAX_HAMMING = 1
UNDISTORT = True

# Circle detection color.
# Use "yellow" for original targets. Use "blue" only if detecting previously annotated blue circles.
DETECT_COLOR = "yellow"

YELLOW_LOWER = [18, 40, 100]
YELLOW_UPPER = [40, 255, 255]

BLUE_LOWER = [95, 80, 40]
BLUE_UPPER = [135, 255, 255]

# ROI crop percentage. Adjust if circles are missed.
CROP_TOP_PERCENT = 48.0
CROP_BOTTOM_PERCENT = 28.0
CROP_LEFT_PERCENT = 6.0
CROP_RIGHT_PERCENT = 6.0
SHOW_ROI_BOX = True

# Contour filters. Lower these if far/small circles are missed.
MIN_CONTOUR_WIDTH = 40
MIN_CONTOUR_HEIGHT = 25
MAX_CONTOUR_WIDTH = 340
MAX_CONTOUR_HEIGHT = 340
MIN_CONTOUR_AREA = 50.0
MIN_POINTS_FOR_ELLIPSE = 5

CLOSE_KERNEL_SIZE = (7, 7)
CLOSE_ITERATIONS = 2

DILATE_KERNEL_SIZE = (5, 5)
DILATE_ITERATIONS = 1

ERODE_KERNEL_SIZE = (3, 3)
ERODE_ITERATIONS = 1

CIRCLE_LINE_THICKNESS = 4

OUTPUT_VIDEO_NAME = "manual_alphabet_labeled_output.mp4"
OUTPUT_TXT_NAME = "manual_alphabet_labeled_coordinates.txt"

# Save still images from the best frame in the video.
# Best frame = frame with highest number of accepted circle detections.
SAVE_BEST_DEBUG_IMAGES = True
BEST_FRAME_IMAGE_NAME = "best_01_annotated_detected_frame.jpg"
BEST_MASK_IMAGE_NAME = "best_02_yellow_mask_roi.jpg"
BEST_CLEAN_MASK_IMAGE_NAME = "best_03_clean_mask_roi.jpg"
BEST_FILTER_DEBUG_IMAGE_NAME = "best_04_filter_debug_accept_reject.jpg"


# ============================================================
# Camera calibration copied from your existing code
# ============================================================

def build_camera_matrix():
    fx, fy = 1.74374549e03, 1.74344675e03
    cx, cy = 5.31071331e02, 9.60993749e02

    K = np.array(
        [
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )

    dist = np.array(
        [
            2.35597446e-01,
            -1.10838235e00,
            2.06356722e-03,
            -7.04828013e-04,
            1.80462654e00,
        ],
        dtype=np.float64,
    )

    return K, dist


# ============================================================
# AprilTag helpers
# ============================================================

def order_corners_tl_tr_br_bl(corners_xy: np.ndarray) -> np.ndarray:
    c = np.asarray(corners_xy, dtype=np.float64).reshape(4, 2)

    s = c[:, 0] + c[:, 1]
    d = c[:, 0] - c[:, 1]

    tl = c[int(np.argmin(s))]
    br = c[int(np.argmax(s))]
    tr = c[int(np.argmin(d))]
    bl = c[int(np.argmax(d))]

    return np.stack([tl, tr, br, bl], axis=0)


def estimate_pose_from_corners(corners_xy, tag_size_m, K, dist):
    s = float(tag_size_m)
    half = s / 2.0

    obj_pts = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float64,
    )

    img_pts_px = order_corners_tl_tr_br_bl(corners_xy)

    und = cv2.undistortPoints(img_pts_px.reshape(-1, 1, 2), K, dist)
    img_pts = und.reshape(-1, 2).astype(np.float64)

    K_eff = np.eye(3, dtype=np.float64)
    dist_eff = None

    def rmse_for(rvec, tvec):
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
        proj = proj.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum((proj - img_pts_px) ** 2, axis=1))))

    best_rvec = None
    best_tvec = None
    best_rmse = float("inf")

    if hasattr(cv2, "solvePnPGeneric"):
        try:
            ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj_pts,
                img_pts,
                K_eff,
                dist_eff,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok and len(rvecs) > 0:
                for r, t in zip(rvecs, tvecs):
                    if float(t.reshape(3)[2]) <= 0:
                        continue
                    err = rmse_for(r, t)
                    if err < best_rmse:
                        best_rvec, best_tvec, best_rmse = r, t, err
        except cv2.error:
            pass

    if best_rvec is None or best_tvec is None:
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            K_eff,
            dist_eff,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError("solvePnP failed")
        best_rvec, best_tvec = rvec, tvec
        best_rmse = rmse_for(best_rvec, best_tvec)

    if hasattr(cv2, "solvePnPRefineLM"):
        try:
            best_rvec, best_tvec = cv2.solvePnPRefineLM(
                obj_pts,
                img_pts,
                K_eff,
                dist_eff,
                best_rvec,
                best_tvec,
            )
            best_rmse = rmse_for(best_rvec, best_tvec)
        except cv2.error:
            pass

    return best_rvec, best_tvec, best_rmse


def pixel_to_table_surface(px, py, K, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)

    cam_origin_ref = -R.T @ t
    ray_cam = np.linalg.inv(K) @ np.array([px, py, 1.0], dtype=np.float64)
    ray_ref = R.T @ ray_cam

    if abs(ray_ref[2]) < 1e-9:
        return None

    k = -cam_origin_ref[2] / ray_ref[2]
    if k < 0:
        return None

    intersection = cam_origin_ref + k * ray_ref
    return float(intersection[0]), float(intersection[1])


def find_reference_tag_pose(detections, ref_tag_id, tag_size_m, K_used, dist_used):
    candidates = []

    for d in detections:
        if int(d.tag_id) != int(ref_tag_id):
            continue
        if hasattr(d, "decision_margin") and float(d.decision_margin) < float(MIN_DECISION_MARGIN):
            continue
        if hasattr(d, "hamming") and int(d.hamming) > int(MAX_HAMMING):
            continue
        candidates.append(d)

    if not candidates:
        return None

    best = max(candidates, key=lambda x: float(getattr(x, "decision_margin", 0.0)))
    corners = np.asarray(best.corners, dtype=np.float64)

    try:
        return estimate_pose_from_corners(
            corners_xy=corners,
            tag_size_m=tag_size_m,
            K=K_used,
            dist=dist_used,
        )
    except Exception:
        return None


# ============================================================
# Circle detection
# ============================================================

RUNTIME_SCALE = 1.0

def compute_runtime_scale(width, height):
    if not AUTO_SCALE_TO_VIDEO_RESOLUTION:
        return 1.0
    sx = float(width) / float(BASE_TUNING_WIDTH)
    sy = float(height) / float(BASE_TUNING_HEIGHT)
    # Use average scale so 3840x2160 becomes exactly 2.0.
    return max(0.25, (sx + sy) / 2.0)

def s_int(value, min_value=1):
    return max(int(round(float(value) * RUNTIME_SCALE)), int(min_value))

def s_float(value):
    return float(value) * float(RUNTIME_SCALE)

def odd_kernel_size(size_tuple):
    w = s_int(size_tuple[0], 1)
    h = s_int(size_tuple[1], 1)
    if w % 2 == 0:
        w += 1
    if h % 2 == 0:
        h += 1
    return (w, h)

def draw_scale():
    return RUNTIME_SCALE if AUTO_SCALE_DRAWING else 1.0

def scale_camera_matrix_to_video(K, video_width, video_height):
    K_scaled = K.copy().astype(np.float64)
    sx = float(video_width) / float(CALIBRATION_FRAME_WIDTH)
    sy = float(video_height) / float(CALIBRATION_FRAME_HEIGHT)
    K_scaled[0, 0] *= sx
    K_scaled[1, 1] *= sy
    K_scaled[0, 2] *= sx
    K_scaled[1, 2] *= sy
    return K_scaled

def apply_frame_rotation(frame):
    if ROTATE_FRAME is None:
        return frame

    mode = str(ROTATE_FRAME).strip().lower()

    if mode in ("clockwise", "cw", "90cw", "90_clockwise"):
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    if mode in ("counterclockwise", "ccw", "90ccw", "90_counterclockwise"):
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    if mode in ("180", "rotate_180"):
        return cv2.rotate(frame, cv2.ROTATE_180)

    return frame


def get_roi_bounds(frame):
    height, width = frame.shape[:2]

    x1 = int(width * CROP_LEFT_PERCENT / 100.0)
    x2 = int(width * (1.0 - CROP_RIGHT_PERCENT / 100.0))

    y1 = int(height * CROP_TOP_PERCENT / 100.0)
    y2 = int(height * (1.0 - CROP_BOTTOM_PERCENT / 100.0))

    x1 = max(0, min(x1, width - 1))
    x2 = max(1, min(x2, width))
    y1 = max(0, min(y1, height - 1))
    y2 = max(1, min(y2, height))

    if x2 <= x1 or y2 <= y1:
        raise ValueError("Invalid ROI crop percentages.")

    return x1, y1, x2, y2


def detect_circle_centers(frame_bgr):
    """
    Detect circle centers and also return debug images.

    Returns:
        output: annotated frame with accepted ellipses
        accepted: list of accepted circles
        mask_full: full-frame binary color mask for visual checking
        clean_full: full-frame cleaned/morphology mask
        filter_debug: frame showing ACCEPT/REJECT boxes for contour filtering
    """
    output = frame_bgr.copy()
    filter_debug = frame_bgr.copy()

    x1, y1, x2, y2 = get_roi_bounds(frame_bgr)
    roi = frame_bgr[y1:y2, x1:x2]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    if DETECT_COLOR == "blue":
        lower = np.array(BLUE_LOWER, dtype=np.uint8)
        upper = np.array(BLUE_UPPER, dtype=np.uint8)
    else:
        lower = np.array(YELLOW_LOWER, dtype=np.uint8)
        upper = np.array(YELLOW_UPPER, dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)

    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, odd_kernel_size(CLOSE_KERNEL_SIZE))
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, odd_kernel_size(DILATE_KERNEL_SIZE))
    kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, odd_kernel_size(ERODE_KERNEL_SIZE))

    clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=CLOSE_ITERATIONS)
    clean = cv2.dilate(clean, kernel_dilate, iterations=DILATE_ITERATIONS)
    clean = cv2.erode(clean, kernel_erode, iterations=ERODE_ITERATIONS)

    # Make full-frame masks so saved debug images match the original frame size.
    mask_full = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    clean_full = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    mask_full[y1:y2, x1:x2] = mask
    clean_full[y1:y2, x1:x2] = clean

    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    accepted = []

    min_w = s_int(MIN_CONTOUR_WIDTH)
    min_h = s_int(MIN_CONTOUR_HEIGHT)
    max_w = s_int(MAX_CONTOUR_WIDTH)
    max_h = s_int(MAX_CONTOUR_HEIGHT)
    min_area = MIN_CONTOUR_AREA * RUNTIME_SCALE * RUNTIME_SCALE

    for cnt in contours:
        area = cv2.contourArea(cnt)
        rx, ry, w, h = cv2.boundingRect(cnt)

        full_rx = int(rx + x1)
        full_ry = int(ry + y1)

        reject_reason = None
        if w < min_w:
            reject_reason = f"REJECT W:{w}<{min_w}"
        elif h < min_h:
            reject_reason = f"REJECT H:{h}<{min_h}"
        elif w > max_w:
            reject_reason = f"REJECT W:{w}>{max_w}"
        elif h > max_h:
            reject_reason = f"REJECT H:{h}>{max_h}"
        elif area < min_area:
            reject_reason = f"REJECT A:{area:.0f}<{min_area:.0f}"
        elif len(cnt) < MIN_POINTS_FOR_ELLIPSE:
            reject_reason = f"REJECT PTS:{len(cnt)}<{MIN_POINTS_FOR_ELLIPSE}"

        if reject_reason is not None:
            cv2.rectangle(filter_debug, (full_rx, full_ry), (full_rx + w, full_ry + h), (0, 0, 255), s_int(1))
            cv2.putText(
                filter_debug,
                reject_reason,
                (full_rx, max(20, full_ry - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35 * draw_scale(),
                (0, 0, 255),
                s_int(1),
            )
            continue

        ellipse = cv2.fitEllipse(cnt)

        cx_full = float(ellipse[0][0]) + x1
        cy_full = float(ellipse[0][1]) + y1

        shifted_ellipse = (
            (ellipse[0][0] + x1, ellipse[0][1] + y1),
            ellipse[1],
            ellipse[2],
        )

        accepted.append(
            {
                "cx_px": cx_full,
                "cy_px": cy_full,
                "ellipse": shifted_ellipse,
                "area": float(area),
                "bbox": (int(full_rx), int(full_ry), int(w), int(h)),
            }
        )

        cv2.rectangle(filter_debug, (full_rx, full_ry), (full_rx + w, full_ry + h), (0, 255, 0), s_int(2))
        cv2.putText(
            filter_debug,
            f"ACCEPT W:{w} H:{h}",
            (full_rx, max(20, full_ry - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35 * draw_scale(),
            (0, 255, 0),
            s_int(1),
        )

    accepted.sort(key=lambda item: (item["cy_px"], item["cx_px"]))

    if SHOW_ROI_BOX:
        for img in (output, filter_debug):
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), s_int(3))
            cv2.putText(
                img,
                "ROI",
                (x1 + 10, y1 + 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0 * draw_scale(),
                (0, 255, 0),
                s_int(2),
            )

    return output, accepted, mask_full, clean_full, filter_debug

# ============================================================
# Manual clicking reference
# ============================================================

def resize_for_screen(frame, max_w=850, max_h=720):
    h, w = frame.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return resized, scale


def collect_manual_reference_points(frame_bgr, reference_path):
    """
    Opens selected reference frame and asks user to click labels one by one.
    Saves pixel positions.
    """
    clicked = {}
    current_index = 0
    display_frame, scale = resize_for_screen(frame_bgr)

    window_name = "Click reference points: A B C D E F G H I J 0"

    def redraw():
        canvas = display_frame.copy()

        # Draw already clicked points
        for label, pt in clicked.items():
            x = int(round(pt["x_px"] * scale))
            y = int(round(pt["y_px"] * scale))
            cv2.circle(canvas, (x, y), 7, (0, 0, 255), -1)
            cv2.putText(
                canvas,
                label,
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )

        if current_index < len(REFERENCE_LABELS):
            label = REFERENCE_LABELS[current_index]
            msg1 = f"Click center of: {label}"
            msg2 = "Left click = mark | Backspace = undo | ESC = exit"
        else:
            msg1 = "All points clicked. Press ENTER or SPACE to save."
            msg2 = "Backspace = undo | ESC = exit"

        header_h = 55
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], header_h), (0, 0, 0), -1)
        cv2.putText(canvas, msg1, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(canvas, msg2, (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        return canvas

    def mouse_callback(event, x, y, flags, param):
        nonlocal current_index

        if event == cv2.EVENT_LBUTTONDOWN and current_index < len(REFERENCE_LABELS):
            label = REFERENCE_LABELS[current_index]
            original_x = float(x) / scale
            original_y = float(y) / scale
            clicked[label] = {"x_px": original_x, "y_px": original_y}
            current_index += 1

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 850, 720)
        cv2.setMouseCallback(window_name, mouse_callback)

        while True:
            canvas = redraw()
            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(30) & 0xFF

            if key == 27:  # ESC
                raise SystemExit("Manual clicking cancelled by user.")

            if key in (8, 127):  # Backspace
                if current_index > 0:
                    current_index -= 1
                    label = REFERENCE_LABELS[current_index]
                    clicked.pop(label, None)

            if key in (13, 32):  # Enter or Space
                if current_index >= len(REFERENCE_LABELS):
                    break

        safe_destroy_windows()

    except cv2.error as exc:
        raise SystemExit(
            "OpenCV GUI window failed.\n\n"
            "Most likely you installed opencv-python-headless.\n"
            "Fix it with:\n"
            "    pip uninstall opencv-python-headless\n"
            "    pip install opencv-python\n\n"
            f"Original OpenCV error:\n{exc}"
        )

    reference_data = {
        "labels": REFERENCE_LABELS,
        "points_px": clicked,
        "use_tag1_coordinates": USE_TAG1_COORDINATES,
        "video_file": VIDEO_FILE,
        "frame_width": int(frame_bgr.shape[1]),
        "frame_height": int(frame_bgr.shape[0]),
    }

    reference_path.parent.mkdir(parents=True, exist_ok=True)
    with reference_path.open("w", encoding="utf-8") as f:
        json.dump(reference_data, f, indent=2)

    print(f"Saved manual reference points: {reference_path}")
    return reference_data


def safe_destroy_windows():
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass


def load_or_collect_reference(frame_bgr, reference_path):
    if reference_path.exists() and not FORCE_RECLICK_REFERENCE:
        with reference_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        print(f"Loaded saved manual reference: {reference_path}")
        return data

    return collect_manual_reference_points(frame_bgr, reference_path)


def convert_reference_to_tag_coords(reference_data, K_used, ref_pose):
    """
    Convert clicked pixel reference points into AprilTag-1 plane coordinates.
    """
    if ref_pose is None:
        return None

    rvec, tvec, rmse = ref_pose

    points_tag = {}

    for label, pt in reference_data["points_px"].items():
        xy = pixel_to_table_surface(
            pt["x_px"],
            pt["y_px"],
            K_used,
            rvec,
            tvec,
        )
        if xy is not None:
            points_tag[label] = {"x": float(xy[0]), "y": float(xy[1])}

    if len(points_tag) == 0:
        return None

    return points_tag


def assign_label_to_detection(item, tag_xy, reference_px, reference_tag):
    """
    Assign nearest manual reference label.
    Prefer Tag-1 coordinates if available.
    """
    if USE_TAG1_COORDINATES and tag_xy is not None and reference_tag is not None:
        x, y = tag_xy
        best_label = "unknown"
        best_dist = float("inf")

        for label, pt in reference_tag.items():
            dx = x - pt["x"]
            dy = y - pt["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < best_dist:
                best_dist = dist
                best_label = label

        if best_dist <= MAX_MATCH_DISTANCE_TAG:
            return best_label, best_dist, "tag1"

        return "unknown", best_dist, "tag1"

    # Pixel fallback
    x = item["cx_px"]
    y = item["cy_px"]

    best_label = "unknown"
    best_dist = float("inf")

    for label, pt in reference_px.items():
        dx = x - pt["x_px"]
        dy = y - pt["y_px"]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < best_dist:
            best_dist = dist
            best_label = label

    if best_dist <= s_float(MAX_MATCH_DISTANCE_PX):
        return best_label, best_dist, "pixel"

    return "unknown", best_dist, "pixel"


# ============================================================
# Main
# ============================================================

def main():
    video_path = Path(VIDEO_FILE)
    if not video_path.exists():
        raise FileNotFoundError(f"Could not open video: {video_path}")

    output_dir = Path(OUTPUT_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)

    reference_path = output_dir / REFERENCE_JSON
    output_video_path = output_dir / OUTPUT_VIDEO_NAME
    output_txt_path = output_dir / OUTPUT_TXT_NAME

    K, dist = build_camera_matrix()

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        fps = 30

    raw_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    raw_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Raw input video resolution from OpenCV: {raw_width}x{raw_height}")
    print(f"Frame rotation mode: {ROTATE_FRAME}")

    detector = Detector(
        families=APRILTAG_FAMILY,
        nthreads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    # Read selected reference frame for manual clicking.
    # This lets you skip bad/blurred first frames and choose a clear board frame.
    cap.set(cv2.CAP_PROP_POS_FRAMES, REFERENCE_FRAME_INDEX)
    ret, first_frame = cap.read()
    if not ret:
        raise RuntimeError(f"Could not read reference frame {REFERENCE_FRAME_INDEX}.")

    first_frame = apply_frame_rotation(first_frame)

    # After rotation, actual processing width/height may change.
    height, width = first_frame.shape[:2]

    global RUNTIME_SCALE
    RUNTIME_SCALE = compute_runtime_scale(width, height)
    print(f"Processing resolution after rotation: {width}x{height}")
    print(f"Runtime scale factor: {RUNTIME_SCALE:.3f}")

    # Scale the camera matrix from calibration resolution to current processing resolution.
    # This is important when using the same calibration values on 4K video.
    K = scale_camera_matrix_to_video(K, width, height)

    K_used = K
    dist_used = dist

    if UNDISTORT:
        K_used, _ = cv2.getOptimalNewCameraMatrix(
            K,
            dist,
            (width, height),
            0.0,
            (width, height),
        )
        dist_used = np.zeros((5,), dtype=np.float64)

    first_frame_for_detection = first_frame
    if UNDISTORT:
        first_frame_for_detection = cv2.undistort(first_frame, K, dist, None, K_used)

    reference_data = load_or_collect_reference(first_frame_for_detection, reference_path)
    reference_px = reference_data["points_px"]

    saved_w = reference_data.get("frame_width")
    saved_h = reference_data.get("frame_height")
    if saved_w is not None and saved_h is not None and (int(saved_w) != width or int(saved_h) != height):
        print("WARNING: Saved manual reference JSON was created for a different resolution.")
        print(f"Saved reference: {saved_w}x{saved_h}, current video: {width}x{height}")
        print("Set FORCE_RECLICK_REFERENCE = True for best 4K accuracy.")

    # Find first-frame tag pose so clicked pixel references can be converted to Tag-1 coords
    gray_first = cv2.cvtColor(first_frame_for_detection, cv2.COLOR_BGR2GRAY)
    detections_first = detector.detect(gray_first)
    first_ref_pose = find_reference_tag_pose(
        detections=detections_first,
        ref_tag_id=REF_TAG_ID,
        tag_size_m=TAG_SIZE_M,
        K_used=K_used,
        dist_used=dist_used,
    )

    reference_tag = None
    if USE_TAG1_COORDINATES:
        reference_tag = convert_reference_to_tag_coords(reference_data, K_used, first_ref_pose)
        if reference_tag is None:
            print("WARNING: Could not convert clicked reference points to Tag-1 coordinates.")
            print("Using pixel matching fallback.")
        else:
            tag_reference_path = output_dir / "manual_reference_points_tag1.json"
            with tag_reference_path.open("w", encoding="utf-8") as f:
                json.dump(reference_tag, f, indent=2)
            print(f"Saved Tag-1 reference points: {tag_reference_path}")

    # Restart video from beginning
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_video_path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open VideoWriter: {output_video_path}")

    frame_count = 0

    # Best still-image output storage.
    # The best frame is selected by the highest number of accepted circles.
    best_detection_count = -1
    best_frame_number = None
    best_annotated_frame = None
    best_mask_img = None
    best_clean_mask_img = None
    best_filter_debug_img = None

    # Store valid coordinates for final average center position per class.
    # For each label, we keep both pixel centers and Tag-1 plane centers when available.
    avg_data = {
        label: {
            "px": [],
            "tag1": [],
        }
        for label in REFERENCE_LABELS
    }

    with output_txt_path.open("w", encoding="utf-8") as f:
        f.write("Manual-reference alphabet circle detection output\n")
        f.write("================================================\n\n")
        f.write(f"Input video: {VIDEO_FILE}\n")
        f.write(f"Input resolution: {width}x{height}\n")
        f.write(f"Runtime scale factor: {RUNTIME_SCALE:.3f}\n")
        f.write(f"Output video: {output_video_path}\n")
        f.write(f"Processed max frames: {'full video' if MAX_FRAMES == 0 else MAX_FRAMES}\n")
        f.write(f"Manual reference frame index: {REFERENCE_FRAME_INDEX}\n")
        f.write(f"Reference labels: {', '.join(REFERENCE_LABELS)}\n")
        f.write(f"Use Tag-1 coordinates: {USE_TAG1_COORDINATES}\n")
        f.write(f"Reference AprilTag ID: {REF_TAG_ID}\n")
        f.write(f"Tag size: {TAG_SIZE_M}\n")
        f.write(f"Save best debug images: {SAVE_BEST_DEBUG_IMAGES}\n")
        f.write(f"Best annotated image: {BEST_FRAME_IMAGE_NAME}\n")
        f.write(f"Best mask image: {BEST_MASK_IMAGE_NAME}\n")
        f.write(f"Best clean mask image: {BEST_CLEAN_MASK_IMAGE_NAME}\n")
        f.write(f"Best filter debug image: {BEST_FILTER_DEBUG_IMAGE_NAME}\n\n")
        f.write("Per-frame detections\n")
        f.write("--------------------\n\n")

        while True:
            ret, frame_bgr = cap.read()
            if not ret:
                break

            frame_bgr = apply_frame_rotation(frame_bgr)

            frame_count += 1

            if MAX_FRAMES > 0 and frame_count > MAX_FRAMES:
                break

            frame_for_detection = frame_bgr
            if UNDISTORT:
                frame_for_detection = cv2.undistort(frame_bgr, K, dist, None, K_used)

            gray = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)

            ref_pose = find_reference_tag_pose(
                detections=detections,
                ref_tag_id=REF_TAG_ID,
                tag_size_m=TAG_SIZE_M,
                K_used=K_used,
                dist_used=dist_used,
            )

            processed_frame, circles_px, mask_img, clean_mask_img, filter_debug_img = detect_circle_centers(frame_for_detection)

            # Save the best-detectable frame for still image output.
            # This is updated before drawing labels, then updated again after all labels are drawn.
            is_current_best_frame = False
            if SAVE_BEST_DEBUG_IMAGES and len(circles_px) > best_detection_count:
                best_detection_count = len(circles_px)
                best_frame_number = frame_count
                best_mask_img = mask_img.copy()
                best_clean_mask_img = clean_mask_img.copy()
                best_filter_debug_img = filter_debug_img.copy()
                is_current_best_frame = True

            if ref_pose is not None:
                ref_rvec, ref_tvec, ref_rmse = ref_pose
                cv2.putText(
                    processed_frame,
                    f"Tag {REF_TAG_ID} detected | circles: {len(circles_px)}",
                    (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )
            else:
                ref_rvec, ref_tvec, ref_rmse = None, None, None
                cv2.putText(
                    processed_frame,
                    f"Tag {REF_TAG_ID} NOT detected | circles: {len(circles_px)}",
                    (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )

            used_labels = set()

            for idx, item in enumerate(circles_px, start=1):
                tag_xy = None
                if ref_pose is not None:
                    tag_xy = pixel_to_table_surface(
                        item["cx_px"],
                        item["cy_px"],
                        K_used,
                        ref_rvec,
                        ref_tvec,
                    )

                label, dist_match, mode = assign_label_to_detection(
                    item=item,
                    tag_xy=tag_xy,
                    reference_px=reference_px,
                    reference_tag=reference_tag,
                )

                # Avoid duplicate assignment if two contours map to same label.
                # Keep first one and mark duplicate as unknown_duplicate.
                if label != "unknown":
                    if label in used_labels:
                        label_to_draw = f"{label}?"
                        label_for_csv = "unknown_duplicate"
                    else:
                        used_labels.add(label)
                        label_to_draw = label
                        label_for_csv = label
                else:
                    label_to_draw = "unknown"
                    label_for_csv = "unknown"

                cx = int(round(item["cx_px"]))
                cy = int(round(item["cy_px"]))

                cv2.ellipse(processed_frame, item["ellipse"], (255, 0, 0), s_int(CIRCLE_LINE_THICKNESS))
                cv2.circle(processed_frame, (cx, cy), 5, (0, 0, 255), -1)

                cv2.putText(
                    processed_frame,
                    label_to_draw,
                    (cx - 25, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.85,
                    (0, 255, 255) if label_for_csv != "unknown" else (0, 0, 255),
                    2,
                )

                cv2.putText(
                    processed_frame,
                    str(idx),
                    (cx - 10, cy + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )

                if tag_xy is not None:
                    tag_x, tag_y = tag_xy
                    coord_text = f"({tag_x:.1f},{tag_y:.1f})"
                else:
                    tag_x, tag_y = None, None
                    coord_text = f"({cx},{cy})"

                cv2.putText(
                    processed_frame,
                    coord_text,
                    (cx + 10, cy + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    2,
                )

                bx, by, bw, bh = item["bbox"]

                # Save this detection for final averaging only if it has a real class label.
                if label_for_csv in avg_data:
                    avg_data[label_for_csv]["px"].append((float(item["cx_px"]), float(item["cy_px"])))
                    if tag_x is not None and tag_y is not None:
                        avg_data[label_for_csv]["tag1"].append((float(tag_x), float(tag_y)))

                f.write(f"Frame {frame_count:06d}\n")
                f.write(f"  Label: {label_for_csv}\n")
                f.write(f"  Match mode: {mode}\n")
                f.write(f"  Match distance: {dist_match:.4f}\n")
                f.write(f"  Pixel center: cx={item['cx_px']:.3f}, cy={item['cy_px']:.3f}\n")

                if tag_x is None or tag_y is None:
                    f.write("  Tag-1 center: NA, NA\n")
                else:
                    f.write(f"  Tag-1 center: x={tag_x:.6f}, y={tag_y:.6f}\n")

                f.write(f"  BBox: x={bx}, y={by}, w={bw}, h={bh}\n")
                f.write(f"  Area: {item['area']:.3f}\n")
                f.write(f"  AprilTag-1 RMSE: {'NA' if ref_rmse is None else f'{ref_rmse:.4f}'}\n")
                f.write("\n")

            # Draw manual reference points on output frame
            for label, pt in reference_px.items():
                x = int(round(pt["x_px"]))
                y = int(round(pt["y_px"]))
                cv2.circle(processed_frame, (x, y), 4, (255, 255, 255), -1)
                cv2.putText(
                    processed_frame,
                    f"ref {label}",
                    (x + 5, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                )

            if SAVE_BEST_DEBUG_IMAGES and is_current_best_frame:
                best_annotated_frame = processed_frame.copy()

            writer.write(processed_frame)

        # Save still images from the best-detectable frame.
        if SAVE_BEST_DEBUG_IMAGES and best_annotated_frame is not None:
            best_frame_path = output_dir / BEST_FRAME_IMAGE_NAME
            best_mask_path = output_dir / BEST_MASK_IMAGE_NAME
            best_clean_path = output_dir / BEST_CLEAN_MASK_IMAGE_NAME
            best_filter_path = output_dir / BEST_FILTER_DEBUG_IMAGE_NAME

            cv2.imwrite(str(best_frame_path), best_annotated_frame)
            cv2.imwrite(str(best_mask_path), best_mask_img)
            cv2.imwrite(str(best_clean_path), best_clean_mask_img)
            cv2.imwrite(str(best_filter_path), best_filter_debug_img)

            print(f"Saved best annotated frame: {best_frame_path}")
            print(f"Saved best yellow mask: {best_mask_path}")
            print(f"Saved best clean mask: {best_clean_path}")
            print(f"Saved best filter debug image: {best_filter_path}")
            print(f"Best frame number: {best_frame_number}, accepted circles: {best_detection_count}")

        # Final average center positions for each class
        f.write("\n\n")
        f.write("Final average center position of each class\n")
        f.write("===========================================\n\n")

        for label in REFERENCE_LABELS:
            px_points = avg_data[label]["px"]
            tag_points = avg_data[label]["tag1"]

            f.write(f"{label}:\n")
            f.write(f"  Detection count: {len(px_points)}\n")

            if len(px_points) > 0:
                px_arr = np.array(px_points, dtype=np.float64)
                avg_px = np.mean(px_arr, axis=0)
                f.write(f"  Average pixel center: cx={avg_px[0]:.3f}, cy={avg_px[1]:.3f}\n")
            else:
                f.write("  Average pixel center: NA, NA\n")

            if len(tag_points) > 0:
                tag_arr = np.array(tag_points, dtype=np.float64)
                avg_tag = np.mean(tag_arr, axis=0)
                f.write(f"  Average Tag-1 center: x={avg_tag[0]:.6f}, y={avg_tag[1]:.6f}\n")
            else:
                f.write("  Average Tag-1 center: NA, NA\n")

            f.write("\n")

        f.write("\nPython CIRCLES list based on average Tag-1 centers\n")
        f.write("=================================================\n")
        f.write("CIRCLES = [\n")
        for label in REFERENCE_LABELS:
            tag_points = avg_data[label]["tag1"]
            if len(tag_points) > 0:
                tag_arr = np.array(tag_points, dtype=np.float64)
                avg_tag = np.mean(tag_arr, axis=0)
                f.write(f'    ({avg_tag[0]:.3f}, {avg_tag[1]:.3f}, 4.550, "{label}"),\n')
            else:
                f.write(f'    # No valid Tag-1 detections for "{label}"\n')
        f.write("]\n")

    cap.release()
    writer.release()
    safe_destroy_windows()

    print("\nDone.")
    print(f"Processed frames: {frame_count}")
    print(f"Saved annotated video: {output_video_path}")
    print(f"Reference frame used for clicking: {REFERENCE_FRAME_INDEX}")
    print(f"Saved text output: {output_txt_path}")
    print(f"Saved/used reference JSON: {reference_path}")


if __name__ == "__main__":
    main()
