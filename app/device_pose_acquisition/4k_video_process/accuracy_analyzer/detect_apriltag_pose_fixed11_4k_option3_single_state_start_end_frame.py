import argparse
import csv
import json
import itertools
import math
import os
import re
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency 'pupil-apriltags'. Install with: pip install -r requirements.txt"
    ) from exc

# ╔══════════════════════════════════════════════════════════════════════╗
# ║  CIRCLES TO DRAW ON THE TABLE SURFACE                              ║
# ║  Each entry is (x_m, y_m, radius_m, label) relative to the        ║
# ║  reference tag center (--circle-ref-tag-id, default tag 1).        ║
# ║  Add / remove / edit lines below, then re-run the script.          ║
# ╚══════════════════════════════════════════════════════════════════════╝
CIRCLES = [
    (-72.392, 31.343, 4.550, "O"),
    (-17.305, 58.900, 4.550, "A"),
    (-37.566, 31.383, 4.550, "B"),
    (-18.236, 3.620, 4.550, "C"),
    (-57.192, 31.383, 4.550, "D"),
    (-37.200, 58.840, 4.550, "E"),
    (-37.997, 3.800, 4.550, "F"),
    (-56.482, 58.840, 4.550, "G"),
    (-57.492, 3.600, 4.550, "H"),
    (-17.950, 31.569, 4.550, "I"),
    (-1.410, 31.526, 4.550, "J"),
    # Add more circles here:
    # ( x_m,   y_m,   radius_m, "label" ),
]


def _sanitize_for_json(obj):
        if obj is None:
                return None
        if isinstance(obj, (str, int, bool)):
                return obj
        if isinstance(obj, float):
                return obj if math.isfinite(obj) else None
        if isinstance(obj, np.integer):
                return int(obj)
        if isinstance(obj, np.floating):
                v = float(obj)
                return v if math.isfinite(v) else None
        if isinstance(obj, np.ndarray):
                return _sanitize_for_json(obj.tolist())
        if isinstance(obj, dict):
                return {str(k): _sanitize_for_json(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
                return [_sanitize_for_json(v) for v in obj]
        return str(obj)


def _resolve_viewer_paths(export_path: str):
        export_path = str(export_path)
        ext = os.path.splitext(export_path)[1].lower()
        if ext == ".json":
                json_path = export_path
                html_path = os.path.join(os.path.dirname(export_path) or ".", "viewer.html")
        else:
                json_path = os.path.join(export_path, "viewer_data.json")
                html_path = os.path.join(export_path, "viewer.html")
        return json_path, html_path


_VIEWER_DATA_ASSIGNMENT_RE = re.compile(
        r"(?P<prefix>const\s+data\s*=\s*)(?P<data>\{.*?\})(?P<suffix>\s*;\s*\n\s*const\s+canvas\s*=)",
        re.DOTALL,
)


def _viewer_template_candidates() -> list[Path]:
        base = Path(__file__).resolve().parent
        return [
                base / "viewer_7371" / "viewer.html",
                base / "viewer_7371" / "viewer2.html",
        ]


def _load_viewer_template_html() -> str:
        for template_path in _viewer_template_candidates():
                if template_path.exists():
                        return template_path.read_text(encoding="utf-8")

        wanted = ", ".join(str(p) for p in _viewer_template_candidates())
        raise FileNotFoundError(f"Viewer template not found. Checked: {wanted}")


def _inject_viewer_data(template_html: str, data_json: str) -> str:
        if "__DATA_JSON__" in template_html:
                return template_html.replace("__DATA_JSON__", data_json, 1)

        match = _VIEWER_DATA_ASSIGNMENT_RE.search(template_html)
        if not match:
                raise ValueError(
                        "Viewer template must contain either __DATA_JSON__ or a 'const data = {...}; const canvas =' block"
                )

        return (
                template_html[: match.start()]
                + match.group("prefix")
                + data_json
                + match.group("suffix")
                + template_html[match.end() :]
        )


def _write_embedded_viewer_html(html_path: str, viewer_data: dict):
        data_json = json.dumps(_sanitize_for_json(viewer_data), ensure_ascii=False)
        template_html = _load_viewer_template_html()
        html = _inject_viewer_data(template_html, data_json)
        Path(html_path).parent.mkdir(parents=True, exist_ok=True)
        Path(html_path).write_text(html, encoding="utf-8")


@contextmanager
def suppress_stderr(enabled: bool = True):
    """Temporarily redirect process stderr to os.devnull.

    Useful for hiding noisy native-library messages such as occasional OpenCV
    solvePnP/IPPE warnings like: "Error, more than one new minima found."
    """
    if not enabled:
        yield
        return

    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_stderr_fd = os.dup(2)
    try:
        os.dup2(devnull_fd, 2)
        yield
    finally:
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


def format_seconds_hms(seconds: float) -> str:
    if not math.isfinite(seconds) or seconds < 0:
        return "--:--:--"
    total = int(round(seconds))
    h = total // 3600
    m = (total % 3600) // 60
    s = total % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def build_camera_matrix():
    fx, fy = 2.76034681e+03, 2.75952005e+03
    cx, cy = 1.05570182e+03, 1.90275625e+03

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
            2.66063938e-01,
            -1.50390150e+00,
            -3.28383631e-03,
            -1.11928861e-03,
            2.94349979e+00,
        ],
        dtype=np.float64,
    )

    return K, dist


def scale_camera_matrix(K: np.ndarray, calib_w: int, calib_h: int, video_w: int, video_h: int) -> np.ndarray:
    """Scales the camera intrinsic matrix to match the current video resolution."""
    if calib_w <= 0 or calib_h <= 0:
        raise ValueError("calib_w and calib_h must be positive")
    K_scaled = K.copy().astype(np.float64)
    sx = float(video_w) / float(calib_w)
    sy = float(video_h) / float(calib_h)
    K_scaled[0, 0] *= sx
    K_scaled[1, 1] *= sy
    K_scaled[0, 2] *= sx
    K_scaled[1, 2] *= sy
    return K_scaled


def choose_camera_matrix_for_video(
    K_raw: np.ndarray,
    video_w: int,
    video_h: int,
    mode: str = "auto",
    calib_w: int = 2160,
    calib_h: int = 3840,
) -> tuple[np.ndarray, str]:
    """Return the camera matrix that matches the current video resolution.

    This file's built-in calibration matrix is already a 4K portrait calibration
    for 2160x3840 video. The previous version always scaled it as if it came
    from 1080x1920, which doubled fx/fy/cx/cy for true 4K videos and caused
    table-circle projection drift.

    mode:
      - none: use K_raw directly
      - scale: scale from --calib-width/--calib-height to the video size
      - auto: use K_raw directly when the video size matches calibration size,
              otherwise scale from --calib-width/--calib-height.
    """
    mode = str(mode).lower().strip()
    if mode == "none":
        return K_raw.copy().astype(np.float64), "none: using calibration matrix without scaling"

    if mode == "scale":
        K_scaled = scale_camera_matrix(K_raw, calib_w, calib_h, video_w, video_h)
        return K_scaled, f"scale: scaled calibration {calib_w}x{calib_h} -> {video_w}x{video_h}"

    # auto
    if int(video_w) == int(calib_w) and int(video_h) == int(calib_h):
        return K_raw.copy().astype(np.float64), f"auto: video matches calibration size {calib_w}x{calib_h}; no scaling"

    # If the raw principal point is already in the middle of the current frame,
    # it is likely already calibrated for this video size. Avoid scaling.
    cx = float(K_raw[0, 2])
    cy = float(K_raw[1, 2])
    cx_ok = 0.35 * float(video_w) <= cx <= 0.65 * float(video_w)
    cy_ok = 0.35 * float(video_h) <= cy <= 0.65 * float(video_h)
    if cx_ok and cy_ok:
        return K_raw.copy().astype(np.float64), (
            f"auto: raw principal point ({cx:.1f},{cy:.1f}) fits video {video_w}x{video_h}; no scaling"
        )

    K_scaled = scale_camera_matrix(K_raw, calib_w, calib_h, video_w, video_h)
    return K_scaled, f"auto: scaled calibration {calib_w}x{calib_h} -> {video_w}x{video_h}"


def _rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rmat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rmat
    T[:3, 3] = tvec.reshape(3)
    return T


def _rodrigues_to_euler_zyx_degrees(rvec: np.ndarray) -> tuple[float, float, float]:
    """Returns yaw,pitch,roll (Z,Y,X) in degrees."""
    rmat, _ = cv2.Rodrigues(rvec)

    # ZYX convention
    sy = math.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)
    singular = sy < 1e-9

    if not singular:
        roll_x = math.atan2(rmat[2, 1], rmat[2, 2])
        pitch_y = math.atan2(-rmat[2, 0], sy)
        yaw_z = math.atan2(rmat[1, 0], rmat[0, 0])
    else:
        roll_x = math.atan2(-rmat[1, 2], rmat[1, 1])
        pitch_y = math.atan2(-rmat[2, 0], sy)
        yaw_z = 0.0

    return (math.degrees(yaw_z), math.degrees(pitch_y), math.degrees(roll_x))


def _order_corners_tl_tr_br_bl(corners_xy: np.ndarray) -> np.ndarray:
    """Return corners ordered as top-left, top-right, bottom-right, bottom-left.

    Works for any convex quadrilateral (including rotated tags).
    """
    c = np.asarray(corners_xy, dtype=np.float64).reshape(4, 2)
    s = c[:, 0] + c[:, 1]
    d = c[:, 0] - c[:, 1]

    tl = c[int(np.argmin(s))]
    br = c[int(np.argmax(s))]
    tr = c[int(np.argmin(d))]
    bl = c[int(np.argmax(d))]

    return np.stack([tl, tr, br, bl], axis=0)


def reprojection_rmse_px(
    corners_xy_px: np.ndarray,
    tag_size_m: float,
    K: np.ndarray,
    dist: np.ndarray | None,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> float:
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

    img_pts = _order_corners_tl_tr_br_bl(corners_xy_px)
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((proj - img_pts) ** 2, axis=1))))


def reprojection_rmse_best_permutation_px(
    corners_xy_px: np.ndarray,
    tag_size_m: float,
    K: np.ndarray,
    dist: np.ndarray | None,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> float:
    """Compute reprojection RMSE allowing any corner ordering.

    Useful when the pose comes from a library with a different tag-corner convention.
    """
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

    corners = np.asarray(corners_xy_px, dtype=np.float64).reshape(4, 2)
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)

    best = float("inf")
    for perm in itertools.permutations(range(4)):
        img = corners[list(perm)]
        err = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=1))))
        if err < best:
            best = err
    return best


def draw_cylinder_attached_to_tag(
    frame_bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray | None,
    rvec: np.ndarray,
    tvec: np.ndarray,
    offset_tag_xyz_m: tuple[float, float, float],
    offset_along_axis_m: float,
    length_m: float,
    diameter_m: float,
    axis: str,
    axis_sign: int,
    mode: str,
    rings: int = 10,
    segments: int = 24,
    color: tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
) -> None:
    """Draw a cylinder rigidly attached to the tag coordinate frame.

    The cylinder is defined in the tag (object) frame.
    The axis is along +/-{X_tag,Y_tag,Z_tag} (chosen by `axis`), controlled by axis_sign (+1 or -1).
    """

    radius = float(diameter_m) / 2.0
    length = float(length_m)
    ox, oy, oz = (float(offset_tag_xyz_m[0]), float(offset_tag_xyz_m[1]), float(offset_tag_xyz_m[2]))
    d = float(offset_along_axis_m)

    rings = max(2, int(rings))
    segments = max(8, int(segments))

    # Precompute angles
    thetas = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    circle_xy = np.stack([np.cos(thetas), np.sin(thetas)], axis=1) * radius

    ring_pts_img = []
    axis_sign = 1 if int(axis_sign) >= 0 else -1

    axis = axis.lower().strip()
    if axis not in {"x", "y", "z"}:
        axis = "z"

    mode = mode.lower().strip()
    if mode not in {"centered", "from-base"}:
        mode = "centered"

    for i in range(rings):
        if mode == "centered":
            # Symmetric about the cylinder center: bottom at -L/2, top at +L/2
            a = (i / (rings - 1) - 0.5) * length
            a0 = d
        else:
            # One-sided from a base plane, extending in +/-axis direction
            a = (i / (rings - 1)) * length * axis_sign
            a0 = axis_sign * d

        if axis == "z":
            pts_tag = np.column_stack(
                [
                    circle_xy[:, 0] + ox,
                    circle_xy[:, 1] + oy,
                    np.full((segments,), (a0 + a) + oz, dtype=np.float64),
                ]
            )
        elif axis == "x":
            pts_tag = np.column_stack(
                [
                    np.full((segments,), (a0 + a) + ox, dtype=np.float64),
                    circle_xy[:, 0] + oy,
                    circle_xy[:, 1] + oz,
                ]
            )
        else:  # axis == "y"
            pts_tag = np.column_stack(
                [
                    circle_xy[:, 0] + ox,
                    np.full((segments,), (a0 + a) + oy, dtype=np.float64),
                    circle_xy[:, 1] + oz,
                ]
            )

        pts_tag = pts_tag.astype(np.float64)

        pts_img, _ = cv2.projectPoints(pts_tag, rvec, tvec, K, dist)
        pts_img = pts_img.reshape(-1, 2)
        ring_pts_img.append(pts_img)

    # Draw rings
    for pts in ring_pts_img:
        poly = np.round(pts).astype(np.int32)
        cv2.polylines(frame_bgr, [poly], isClosed=True, color=color, thickness=thickness)

    # Draw longitudinal lines (at 4 angles)
    for k in [0, segments // 4, segments // 2, (3 * segments) // 4]:
        for i in range(rings - 1):
            p1 = tuple(np.round(ring_pts_img[i][k]).astype(int))
            p2 = tuple(np.round(ring_pts_img[i + 1][k]).astype(int))
            cv2.line(frame_bgr, p1, p2, color, thickness)


def choose_cylinder_axis_sign(
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray | None,
    base_offset_xyz_m: tuple[float, float, float],
    offset_along_axis_m: float,
    length_m: float,
    axis: str,
    facing: str,
) -> int:
    """Pick the cylinder axis sign (+1 or -1).

    Modes:
    - facing='away'/'toward': choose by camera-depth (OpenCV camera +Z is forward/away).
    - facing='image-down'/'image-up': choose by projected pixel Y direction.
    """
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)

    ox, oy, oz = (
        float(base_offset_xyz_m[0]),
        float(base_offset_xyz_m[1]),
        float(base_offset_xyz_m[2]),
    )
    d = float(offset_along_axis_m)
    L = float(length_m)
    axis = axis.lower().strip()
    if axis not in {"x", "y", "z"}:
        axis = "z"

    def end_tag_for(sign: int) -> np.ndarray:
        if axis == "z":
            base_tag = np.array([ox, oy, oz + sign * d], dtype=np.float64)
            end_tag = base_tag + np.array([0.0, 0.0, sign * L], dtype=np.float64)
        elif axis == "x":
            base_tag = np.array([ox + sign * d, oy, oz], dtype=np.float64)
            end_tag = base_tag + np.array([sign * L, 0.0, 0.0], dtype=np.float64)
        else:  # axis == "y"
            base_tag = np.array([ox, oy + sign * d, oz], dtype=np.float64)
            end_tag = base_tag + np.array([0.0, sign * L, 0.0], dtype=np.float64)
        return end_tag

    def end_cam_for(sign: int) -> np.ndarray:
        end_tag = end_tag_for(sign)
        return (R @ end_tag) + t

    def end_img_y_for(sign: int) -> float:
        end_tag = end_tag_for(sign).reshape(1, 1, 3)
        pt, _ = cv2.projectPoints(end_tag, rvec, tvec, K, dist)
        return float(pt.reshape(2)[1])

    facing = facing.lower().strip()
    if facing in {"image-down", "down"}:
        y_pos = end_img_y_for(+1)
        y_neg = end_img_y_for(-1)
        return +1 if y_pos >= y_neg else -1
    if facing in {"image-up", "up"}:
        y_pos = end_img_y_for(+1)
        y_neg = end_img_y_for(-1)
        return +1 if y_pos <= y_neg else -1

    z_pos = float(end_cam_for(+1)[2])
    z_neg = float(end_cam_for(-1)[2])
    if facing == "away":
        return +1 if z_pos >= z_neg else -1
    return +1 if z_pos <= z_neg else -1


def estimate_pose_from_corners(
    corners_xy: np.ndarray,
    tag_size_m: float,
    K: np.ndarray,
    dist: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Solve PnP for a square tag.

    corners_xy: (4,2) pixel coordinates in the same order as the object points.
    Returns (rvec, tvec, reprojection_rmse_px).
    """
    s = float(tag_size_m)
    half = s / 2.0

    # Object points in tag coordinate frame (Z=0 plane).
    # Corner order is chosen to match pupil_apriltags corner order (clockwise).
    # If your axes look mirrored, swap ordering here.
    obj_pts = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float64,
    )

    img_pts_px = _order_corners_tl_tr_br_bl(corners_xy)

    # Undistort to normalized image coordinates for more stable PnP.
    # undistortPoints returns points in normalized camera coordinates when P is None.
    und = cv2.undistortPoints(img_pts_px.reshape(-1, 1, 2), K, dist)
    img_pts = und.reshape(-1, 2).astype(np.float64)
    K_eff = np.eye(3, dtype=np.float64)
    dist_eff = None

    def rmse_for(rt: tuple[np.ndarray, np.ndarray]) -> float:
        r, t = rt
        proj, _ = cv2.projectPoints(obj_pts, r, t, K, dist)
        proj = proj.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum((proj - img_pts_px) ** 2, axis=1))))

    # Prefer IPPE_SQUARE, but explicitly pick the best of the (up to) 2 solutions.
    # For planar squares there is a common "flip" ambiguity; prefer solutions with tvec_z > 0.
    rvec = None
    tvec = None
    best_rmse = float("inf")
    best_rmse_posz = float("inf")
    rvec_posz = None
    tvec_posz = None

    if hasattr(cv2, "solvePnPGeneric"):
        try:
            with suppress_stderr(True):
                ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj_pts,
                img_pts,
                K_eff,
                dist_eff,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok and len(rvecs) > 0:
                for r, t in zip(rvecs, tvecs):
                    err = rmse_for((r, t))
                    if err < best_rmse:
                        rvec, tvec, best_rmse = r, t, err
                    if float(t.reshape(3)[2]) > 0 and err < best_rmse_posz:
                        rvec_posz, tvec_posz, best_rmse_posz = r, t, err
        except cv2.error:
            pass

    if rvec_posz is not None and tvec_posz is not None:
        rvec, tvec, best_rmse = rvec_posz, tvec_posz, best_rmse_posz

    if rvec is None or tvec is None:
        ok, r, t = cv2.solvePnP(
            obj_pts,
            img_pts,
            K_eff,
            dist_eff,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError("solvePnP failed")
        rvec, tvec = r, t
        best_rmse = rmse_for((rvec, tvec))

    # Refine (if available) to reduce jitter / avoid poor local minima.
    if hasattr(cv2, "solvePnPRefineLM"):
        try:
            rvec, tvec = cv2.solvePnPRefineLM(
                obj_pts,
                img_pts,
                K_eff,
                dist_eff,
                rvec,
                tvec,
            )
            best_rmse = rmse_for((rvec, tvec))
        except cv2.error:
            pass

    # If we ended up with a behind-camera solution, try to switch to a positive-Z IPPE candidate.
    if float(tvec.reshape(3)[2]) <= 0 and hasattr(cv2, "solvePnPGeneric"):
        try:
            with suppress_stderr(True):
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
                    err = rmse_for((r, t))
                    if err < best_rmse:
                        rvec, tvec, best_rmse = r, t, err
        except cv2.error:
            pass

    return rvec, tvec, best_rmse


def _order_corners_clockwise(centers: list[np.ndarray]) -> list[np.ndarray]:
    """Order 2D points in clockwise order starting from top-left."""
    pts = np.array(centers, dtype=np.float64).reshape(-1, 2)
    centroid = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - centroid[1], pts[:, 0] - centroid[0])
    order = np.argsort(angles)
    ordered = pts[order]
    # Rotate so that top-left (smallest x+y sum) comes first
    sums = ordered[:, 0] + ordered[:, 1]
    start = int(np.argmin(sums))
    ordered = np.roll(ordered, -start, axis=0)
    return [ordered[i] for i in range(len(ordered))]


def draw_table_outline(
    frame_bgr: np.ndarray,
    table_detections: dict[int, "Detection"],
    color: tuple[int, int, int] = (255, 0, 255),
    thickness: int = 3,
) -> None:
    """Draw the table outline using the outermost corners of 4 detected AprilTags.

    For each tag at a table corner, the outermost corner (furthest from the
    centroid of all tag centers) is selected. The four outermost corners are
    then connected in order to form the table boundary.
    """
    if len(table_detections) < 2:
        return

    # Compute centroid of all tag centers
    centers = [np.asarray(d.center, dtype=np.float64) for d in table_detections.values()]
    centroid = np.mean(centers, axis=0)

    # For each tag, pick the corner furthest from the centroid
    outer_corners = []
    for d in table_detections.values():
        corners = np.asarray(d.corners, dtype=np.float64).reshape(-1, 2)
        dists = np.linalg.norm(corners - centroid, axis=1)
        outer_corners.append(corners[int(np.argmax(dists))])

    # Order the outer corners clockwise
    ordered = _order_corners_clockwise(outer_corners)
    poly = np.array(ordered, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(frame_bgr, [poly], isClosed=True, color=color, thickness=thickness)

    # Draw small circles at each outer corner for visibility
    for pt in ordered:
        cv2.circle(frame_bgr, (int(pt[0]), int(pt[1])), 6, color, -1)


def draw_circle_on_table_surface(
    frame_bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray | None,
    rvec: np.ndarray,
    tvec: np.ndarray,
    x_offset_m: float,
    y_offset_m: float,
    radius_m: float,
    label: str = "",
    segments: int = 64,
    color: tuple[int, int, int] = (0, 255, 255),
    thickness: int = 2,
) -> None:
    """Draw a circle on the tag's Z=0 plane (table surface).

    The circle center is at (x_offset_m, y_offset_m) in the reference tag's
    coordinate frame, lying on the table surface (Z=0).
    """
    thetas = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=True)
    circle_pts = np.zeros((len(thetas), 3), dtype=np.float64)
    circle_pts[:, 0] = x_offset_m + radius_m * np.cos(thetas)
    circle_pts[:, 1] = y_offset_m + radius_m * np.sin(thetas)
    # Z = 0: on the table surface

    pts_img, _ = cv2.projectPoints(circle_pts, rvec, tvec, K, dist)
    pts_img = np.round(pts_img.reshape(-1, 2)).astype(np.int32)
    cv2.polylines(frame_bgr, [pts_img], isClosed=True, color=color, thickness=thickness)

    # Draw label at the circle center
    if label:
        center_3d = np.array([[x_offset_m, y_offset_m, 0.0]], dtype=np.float64)
        center_img, _ = cv2.projectPoints(center_3d, rvec, tvec, K, dist)
        cx, cy = int(round(center_img[0, 0, 0])), int(round(center_img[0, 0, 1]))
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.2
        font_thickness = 3
        (tw, th), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
        # Center the text on the projected point
        tx = cx - tw // 2
        ty = cy + th // 2
        cv2.putText(frame_bgr, label, (tx, ty), font, font_scale, color, font_thickness, cv2.LINE_AA)


def pixel_to_table_surface(
    px: float,
    py: float,
    K: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> tuple[float, float] | None:
    """Back-project a pixel (px, py) onto the reference tag's Z=0 plane.

    Returns (x_ref, y_ref) in the tag's coordinate frame, or None if
    the ray is parallel to the plane.
    """
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    # Camera origin in ref frame
    cam_origin_ref = -R.T @ t
    # Ray direction in camera frame (normalized coords)
    ray_cam = np.linalg.inv(K) @ np.array([px, py, 1.0], dtype=np.float64)
    # Ray direction in ref frame
    ray_ref = R.T @ ray_cam
    # Intersect with Z=0 plane
    if abs(ray_ref[2]) < 1e-9:
        return None
    k = -cam_origin_ref[2] / ray_ref[2]
    if k < 0:
        return None  # intersection is behind the camera
    intersection = cam_origin_ref + k * ray_ref
    return (float(intersection[0]), float(intersection[1]))


def circle_intersection_area(r0: float, r1: float, d: float) -> float:
    """Area of overlap between two circles."""
    r0 = float(max(0.0, r0))
    r1 = float(max(0.0, r1))
    d = float(max(0.0, d))

    if r0 <= 0.0 or r1 <= 0.0:
        return 0.0
    if d >= r0 + r1:
        return 0.0

    if d <= abs(r0 - r1):
        return math.pi * min(r0, r1) ** 2

    term0 = r0 * r0 * math.acos((d * d + r0 * r0 - r1 * r1) / (2.0 * d * r0))
    term1 = r1 * r1 * math.acos((d * d + r1 * r1 - r0 * r0) / (2.0 * d * r1))
    term2 = 0.5 * math.sqrt(
        max(
            0.0,
            (-d + r0 + r1)
            * (d + r0 - r1)
            * (d - r0 + r1)
            * (d + r0 + r1),
        )
    )
    return term0 + term1 - term2


def compute_cylinder_circle_overlap_percentages(
    circles: list[tuple[float, float, float, str]],
    cylinder_pos_ref_xy: tuple[float, float] | None,
    cylinder_radius_m: float,
) -> list[tuple[str, float]]:
    """Return overlap percent of each table circle covered by the cylinder footprint.

    Percentage is intersection_area / table_circle_area * 100.
    """
    results: list[tuple[str, float]] = []
    if cylinder_pos_ref_xy is None or cylinder_radius_m <= 0:
        for _, _, _, label in circles:
            results.append((label, 0.0))
        return results

    cx, cy = cylinder_pos_ref_xy
    for ox, oy, orad, label in circles:
        d = math.hypot(cx - ox, cy - oy)
        inter = circle_intersection_area(cylinder_radius_m, orad, d)
        denom = math.pi * max(orad, 1e-12) ** 2
        pct = 100.0 * inter / denom if denom > 0 else 0.0
        pct = max(0.0, min(100.0, pct))
        results.append((label, pct))
    return results


def is_reasonable_cylinder_position(
    cylinder_pos_ref_xy: tuple[float, float] | None,
    circles: list[tuple[float, float, float, str]],
    extra_margin_m: float = 20.0,
) -> bool:
    """Reject obviously-bad cylinder XY values that would blow up the minimap scale.

    When pose estimation temporarily fails, the projected cylinder position can jump far away
    from the table. In that case we keep the last valid position instead of redrawing the
    minimap with a wildly incorrect scale.
    """
    if cylinder_pos_ref_xy is None or not circles:
        return False

    x, y = cylinder_pos_ref_xy
    if not (math.isfinite(x) and math.isfinite(y)):
        return False

    xs = [c[0] for c in circles]
    ys = [c[1] for c in circles]
    rs = [c[2] for c in circles]
    margin = max(extra_margin_m, (max(rs) if rs else 0.0) * 4.0)

    return (min(xs) - margin) <= x <= (max(xs) + margin) and (min(ys) - margin) <= y <= (max(ys) + margin)


def compute_cylinder_contact_point_tag(
    offset_tag_xyz_m: tuple[float, float, float],
    offset_along_axis_m: float,
    length_m: float,
    axis: str,
    axis_sign: int,
    mode: str,
) -> np.ndarray:
    """Return the final cylinder footprint/contact point in the tracked-tag frame.

    This is the single geometry source used for the minimap, overlap values,
    and viewer export. The video wireframe uses the same offset values for its
    rendered cylinder, so all outputs refer to the same final cylinder state.
    """
    ox, oy, oz = (float(offset_tag_xyz_m[0]), float(offset_tag_xyz_m[1]), float(offset_tag_xyz_m[2]))
    d = float(offset_along_axis_m)
    L = float(length_m)
    sign = 1 if int(axis_sign) >= 0 else -1
    axis = str(axis).lower().strip()
    if axis not in {"x", "y", "z"}:
        axis = "z"

    mode = str(mode).lower().strip()
    if mode == "from-base":
        base_axis_offset = sign * d
    else:
        # centered mode: the footprint/contact end is half a length opposite
        # the cylinder extension direction.
        base_axis_offset = d - (sign * L / 2.0)

    if axis == "x":
        return np.array([ox + base_axis_offset, oy, oz], dtype=np.float64)
    if axis == "y":
        return np.array([ox, oy + base_axis_offset, oz], dtype=np.float64)
    return np.array([ox, oy, oz + base_axis_offset], dtype=np.float64)


def build_final_cylinder_state(
    cyl_contact_cam: np.ndarray | None,
    ref_rvec: np.ndarray | None,
    ref_tvec: np.ndarray | None,
    K: np.ndarray,
    dist: np.ndarray | None,
    cylinder_radius_m: float,
) -> dict | None:
    """Build one canonical cylinder state relative to the table/reference tag.

    The returned `table_xy` is the only XY value used by the top-view preview,
    overlap calculation, and JSON/viewer export. `ref_xyz` is kept for height
    only, so the 3D viewer cannot drift sideways from the top-view footprint.
    """
    if cyl_contact_cam is None or ref_rvec is None or ref_tvec is None:
        return None

    try:
        contact_cam = np.asarray(cyl_contact_cam, dtype=np.float64).reshape(3)
        R_ref, _ = cv2.Rodrigues(ref_rvec)
        t_ref = np.asarray(ref_tvec, dtype=np.float64).reshape(3)
        ref_xyz_arr = R_ref.T @ (contact_cam - t_ref)

        table_xy: tuple[float, float] | None = None
        try:
            # Project the final contact point to the image and ray-cast that pixel
            # back onto the reference table plane. This makes the top-view marker
            # directly comparable with the drawn table circles.
            cyl_img, _ = cv2.projectPoints(
                contact_cam.reshape(1, 1, 3),
                np.zeros((3, 1), dtype=np.float64),
                np.zeros((3, 1), dtype=np.float64),
                K,
                dist,
            )
            cyl_px, cyl_py = cyl_img.reshape(2)
            table_xy = pixel_to_table_surface(float(cyl_px), float(cyl_py), K, ref_rvec, ref_tvec)
        except Exception:
            table_xy = None

        if table_xy is None:
            table_xy = (float(ref_xyz_arr[0]), float(ref_xyz_arr[1]))

        return {
            "contact_cam": tuple(float(v) for v in contact_cam),
            "ref_xyz": tuple(float(v) for v in ref_xyz_arr),
            "table_xy": (float(table_xy[0]), float(table_xy[1])),
            "radius_m": float(cylinder_radius_m),
        }
    except Exception:
        return None


def draw_2d_topdown_view(
    frame_bgr: np.ndarray,
    circles: list[tuple[float, float, float, str]],
    cylinder_pos_ref_xy: tuple[float, float] | None = None,
    cylinder_radius_m: float = 0.0,
    minimap_size: int = 375,
    pad_px: int = 15,
    topview_rotation: int = 180,
) -> None:
    """Draw a 2D bird's-eye minimap of the table surface in the top-right corner."""
    if not circles and cylinder_pos_ref_xy is None:
        return

    all_x = [c[0] for c in circles]
    all_y = [c[1] for c in circles]
    all_r = [c[2] * math.sqrt(2.0) for c in circles]

    if cylinder_pos_ref_xy is not None:
        all_x.append(cylinder_pos_ref_xy[0])
        all_y.append(cylinder_pos_ref_xy[1])
        all_r.append(cylinder_radius_m)

    if not all_x:
        return

    pad_m = max(all_r) * 2.5 if all_r else 0.02
    x_min = min(all_x) - pad_m
    x_max = max(all_x) + pad_m
    y_min = min(all_y) - pad_m
    y_max = max(all_y) + pad_m

    x_range = x_max - x_min
    y_range = y_max - y_min
    if x_range < 1e-6 or y_range < 1e-6:
        return

    scale = min(minimap_size / x_range, minimap_size / y_range) * 0.85
    map_w = int(x_range * scale) + 2 * pad_px
    map_h = int(y_range * scale) + 2 * pad_px + 45  # extra for title

    minimap = np.full((map_h, map_w, 3), (30, 30, 30), dtype=np.uint8)
    cv2.rectangle(minimap, (0, 0), (map_w - 1, map_h - 1), (160, 160, 160), 1)
    cv2.putText(minimap, "Top View", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (220, 220, 220), 2, cv2.LINE_AA)

    y_off = 45  # vertical offset for the title bar

    def to_px(xm: float, ym: float) -> tuple[int, int]:
        rotation = int(topview_rotation) % 360

        if rotation == 0:
            px = int((xm - x_min) * scale) + pad_px
            py = int((y_max - ym) * scale) + pad_px + y_off
        elif rotation == 90:
            # 90° clockwise: (x, y) -> (y, -x)
            px = int((ym - y_min) * scale) + pad_px
            py = int((x_max - xm) * scale) + pad_px + y_off
        elif rotation == 180:
            # 180° clockwise: (x, y) -> (-x, -y)
            px = int((x_max - xm) * scale) + pad_px
            py = int((ym - y_min) * scale) + pad_px + y_off
        elif rotation == 270:
            # 270° clockwise, then flipped left-right to match the camera/video layout
            # expected by this project: A/E/G on the right side and C/F/H on the left.
            px = int((ym - y_min) * scale) + pad_px
            py = int((xm - x_min) * scale) + pad_px + y_off
        else:
            px = int((x_max - xm) * scale) + pad_px
            py = int((ym - y_min) * scale) + pad_px + y_off
        return px, py

    def r_to_px(rm: float) -> int:
        return max(1, int(rm * scale))

    # Outer circles (green)
    for cx_m, cy_m, cr_m, _ in circles:
        px, py = to_px(cx_m, cy_m)
        cv2.circle(minimap, (px, py), r_to_px(cr_m * math.sqrt(2.0)),
                   (0, 200, 0), 1, cv2.LINE_AA)

    # Inner circles (yellow) with labels
    for cx_m, cy_m, cr_m, clabel in circles:
        px, py = to_px(cx_m, cy_m)
        cv2.circle(minimap, (px, py), r_to_px(cr_m),
                   (0, 255, 255), 1, cv2.LINE_AA)
        if clabel:
            fs = 0.7
            (tw, th), _ = cv2.getTextSize(clabel, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)
            cv2.putText(minimap, clabel, (px - tw // 2, py + th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 255, 255), 2, cv2.LINE_AA)

    # Cylinder footprint (red)
    if cylinder_pos_ref_xy is not None and cylinder_radius_m > 0:
        cpx, cpy = to_px(cylinder_pos_ref_xy[0], cylinder_pos_ref_xy[1])
        cr_px = r_to_px(cylinder_radius_m)
        cv2.circle(minimap, (cpx, cpy), cr_px, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.circle(minimap, (cpx, cpy), 4, (0, 0, 255), -1)
        cv2.putText(minimap, "CYL", (cpx - 25, cpy - cr_px - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

    # Overlay on top-right corner with alpha blending
    fh, fw = frame_bgr.shape[:2]
    x_start = max(0, fw - map_w - 10)
    y_start = 10
    if y_start + map_h > fh:
        return

    roi = frame_bgr[y_start:y_start + map_h, x_start:x_start + map_w]
    blended = cv2.addWeighted(minimap, 0.8, roi, 0.2, 0)
    frame_bgr[y_start:y_start + map_h, x_start:x_start + map_w] = blended


def main():
    parser = argparse.ArgumentParser(
        description="Detect AprilTag in a video and estimate 3D pose (rvec/tvec) using a calibrated camera matrix."
    )
    parser.add_argument(
        "--video",
        type=str,
        default="IMG_7365.MOV",
        help="Path to input video (default: IMG_7365.MOV)",
    )
    parser.add_argument(
        "--tag-id",
        type=int,
        default=5,
        help="AprilTag id to track (default: 5)",
    )
    parser.add_argument(
        "--family",
        type=str,
        default="tag16h5",
        help="Tag family (default: tag16h5)",
    )
    parser.add_argument(
        "--tag-size-m",
        type=float,
        default=7.3,
        help="Physical tag size (edge length) in meters, e.g. 0.036 for 36mm",
    )
    parser.add_argument(
        "--pose-method",
        choices=["detector", "pnp"],
        default="detector",
        help="Pose estimation method: 'detector' (undistort + apriltags pose) or 'pnp' (solvePnP) (default: detector)",
    )
    parser.add_argument(
        "--undistort",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Undistort frames before detection/pose (default: on)",
    )
    parser.add_argument(
        "--camera-scale-mode",
        choices=["auto", "scale", "none"],
        default="auto",
        help=(
            "How to adapt the built-in camera matrix to the video resolution. "
            "Default auto assumes the built-in matrix is calibrated for --calib-width x --calib-height "
            "and avoids the old double-scaling problem."
        ),
    )
    parser.add_argument(
        "--calib-width",
        type=int,
        default=2160,
        help="Width of the images used for this built-in calibration matrix (default: 2160 for 4K portrait)",
    )
    parser.add_argument(
        "--calib-height",
        type=int,
        default=3840,
        help="Height of the images used for this built-in calibration matrix (default: 3840 for 4K portrait)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="If >0, process only this many frames from --start-frame",
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        default=1,
        help="1-based frame number to start processing from (default: 1)",
    )
    parser.add_argument(
        "--end-frame",
        type=int,
        default=0,
        help="1-based frame number to stop processing at, inclusive. 0 means process until video ends.",
    )
    parser.add_argument(
        "--annotated-out",
        type=str,
        default="",
        help="If set, path to write annotated video (e.g. annotated.mp4)",
    )
    parser.add_argument(
        "--csv-out",
        type=str,
        default="",
        help="If set, write CSV output to this path. Leave empty to disable CSV writing.",
    )
    parser.add_argument(
        "--min-decision-margin",
        type=float,
        default=30.0,
        help="Ignore detections with decision_margin below this (default: 30)",
    )
    parser.add_argument(
        "--tracked-tag-min-decision-margin",
        type=float,
        default=40.0,
        help="Ignore tracked-tag detections with decision_margin below this (default: 40)",
    )
    parser.add_argument(
        "--max-hamming",
        type=int,
        default=0.0,
        help="Ignore detections with hamming greater than this (default: 0)",
    )
    parser.add_argument(
        "--max-rmse-px",
        type=float,
        default=2.0,
        help="Only draw overlays (axes/cylinder/text) if reprojection RMSE <= this (default: 2px)",
    )
    parser.add_argument(
        "--cylinder-length-m",
        type=float,
        default=18.0,
        help="Cylinder length in meters (default: 18.0 = 18mm)",
    )
    parser.add_argument(
        "--cylinder-diameter-m",
        type=float,
        default=9.0,
        help="Cylinder diameter in meters (default: 9.0 = 9mm)",
    )
    parser.add_argument(
        "--cylinder-mode",
        choices=["centered", "from-base"],
        default="from-base",
        help="How to place cylinder along its axis: centered (±L/2) or from-base (0..L) (default: centered)",
    )
    parser.add_argument(
        "--cylinder-axis-sign",
        choices=["auto", "+", "-"],
        default="auto",
        help="Force cylinder direction along the chosen axis: auto / + / - (default: auto)",
    )
    parser.add_argument(
        "--cylinder-anchor",
        choices=["center", "tag-bottom"],
        default="center",
        help="Anchor cylinder position: center (use offsets as-is) or tag-bottom (anchor to the tag bottom edge in the image; only for --cylinder-axis y) (default: center)",
    )
    parser.add_argument(
        "--cylinder-bias-m",
        type=float,
        default=0.0,
        help="Bias distance from the anchor line (meters). For tag-bottom, positive pushes further past the bottom edge (default: 0.002 = 2mm)",
    )
    parser.add_argument(
        "--cylinder-axis",
        choices=["x", "y", "z"],
        default="y",
        help="Cylinder axis in tag frame: x=red, y=green, z=blue (default: z; perpendicular to tag plane)",
    )
    parser.add_argument(
        "--cylinder-offset-x-m",
        type=float,
        default=0.0,
        help="Cylinder base center offset in tag X (meters) (default: 0)",
    )
    parser.add_argument(
        "--cylinder-offset-y-m",
        type=float,
        default=0.0,
        help="Cylinder base center offset in tag Y (meters) (default: 0.006 = 6mm below tag center in-plane)",
    )
    parser.add_argument(
        "--cylinder-offset-z-m",
        type=float,
        default=0.0,
        help="Cylinder base center offset in tag Z (meters) (default: 0)",
    )
    parser.add_argument(
        "--cylinder-draw-offset-y-m",
        type=float,
        default=6.0,
        help=(
            "Extra offset in tag Y for the drawn cylinder outline. By default this same "
            "offset is also applied to the 2D top view, overlap calculation, and viewer export "
            "so all outputs show the same cylinder footprint. Use --no-sync-draw-offset-to-topview "
            "to restore the old visual-only behavior."
        ),
    )
    parser.add_argument(
        "--sync-draw-offset-to-topview",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Apply --cylinder-draw-offset-y-m to the cylinder geometry used by the 2D top view, "
            "overlap percentages, and exported JSON/viewer data (default: on). Disable with "
            "--no-sync-draw-offset-to-topview for the old behavior where the offset only moved "
            "the drawn video overlay."
        ),
    )
    parser.add_argument(
        "--cylinder-offset-along-axis-m",
        type=float,
        default=0.0,
        help="Additional offset along cylinder axis (meters) (default: 0)",
    )
    parser.add_argument(
        "--cylinder-facing",
        choices=["away", "toward", "image-down", "image-up"],
        default="away",
        help="Choose cylinder direction (+axis vs -axis). 'away' uses depth (good for bottom/back side); 'image-down' uses pixel Y (default: away)",
    )
    parser.add_argument(
        "--display",
        action="store_true",
        help="Show a preview window while processing",
    )
    parser.add_argument(
        "--table-tag-ids",
        type=int,
        nargs="*",
        default=[1,2,3,4],
        help="Tag IDs marking the four corners of a table (e.g. --table-tag-ids 1 2 3 4). "
             "When provided, the table outline is drawn by connecting the outermost corners of each tag.",
    )
    parser.add_argument(
        "--table-color",
        type=int,
        nargs=3,
        default=[255, 0, 255],
        metavar=("B", "G", "R"),
        help="BGR color for the table outline (default: 255 0 255 = magenta)",
    )
    parser.add_argument(
        "--table-thickness",
        type=int,
        default=3,
        help="Line thickness for the table outline (default: 3)",
    )
    parser.add_argument(
        "--table-min-decision-margin",
        type=float,
        default=10.0,
        help="Minimum decision margin for table corner tags (independent of --min-decision-margin) (default: 10)",
    )
    parser.add_argument(
        "--table-max-hamming",
        type=int,
        default=1,
        help="Maximum hamming for table corner tags (independent of --max-hamming) (default: 1)",
    )
    parser.add_argument(
        "--table-quad-decimate",
        type=float,
        default=1.0,
        help="Quad decimate for the table tag detector (default: 1.0)",
    )
    parser.add_argument(
        "--table-tag-size-m",
        type=float,
        default=7.3,
        help="Physical size (edge length) of table corner tags in meters. Required for drawing circles on the table surface.",
    )
    parser.add_argument(
        "--circle",
        type=float,
        nargs=3,
        metavar=("X_M", "Y_M", "RADIUS_M"),
        action="append",
        default=None,
        help="Draw a circle on the table surface at (x, y) offset from the reference tag center, "
             "with the given radius, all in meters. Can be specified multiple times. "
             "Example: --circle -0.0925 0.156 0.0225 --circle 0.1 0.2 0.03",
    )
    parser.add_argument(
        "--circle-ref-tag-id",
        type=int,
        default=1,
        help="Tag ID to use as the reference origin for circle placement (default: 1)",
    )
    parser.add_argument(
        "--circle-color",
        type=int,
        nargs=3,
        default=[0, 255, 255],
        metavar=("B", "G", "R"),
        help="BGR color for the circle (default: 0 255 255 = yellow)",
    )
    parser.add_argument(
        "--circle-thickness",
        type=int,
        default=1,
        help="Line thickness for the circle (default: 1)",
    )
    parser.add_argument(
        "--include-zero-circle",
        action="store_true",
        help=(
            "Also draw the label '0' circle from CIRCLES. By default it is skipped because "
            "the current manual 4K run confused 0 with D / an old coordinate."
        ),
    )
    parser.add_argument("--crop-top-percent", type=float, default=30,
                        help="Crop this percent from the top of the frame (default: 30)")
    parser.add_argument("--crop-bottom-percent", type=float, default=12,
                        help="Crop this percent from the bottom of the frame (default: 20)")
    parser.add_argument("--crop-left-percent", type=float, default=3.0,
                        help="Crop this percent from the left of the frame (default: 0)")
    parser.add_argument("--crop-right-percent", type=float, default=5.0,
                        help="Crop this percent from the right of the frame (default: 0)")
    parser.add_argument("--crop-resize", action="store_true",
                        help="Resize cropped ROI back to original frame size")
    parser.add_argument(
        "--freeze-after",
        type=int,
        default=20,
        help="Freeze table outline and circles after this many frames using averaged positions (default: 20). Set 0 to disable.",
    )
    parser.add_argument(
        "--topview-rotation",
        type=int,
        choices=[0, 90, 180, 270],
        default=270,
        help="Rotation of the 2D top view in degrees clockwise (default: 180)",
    )
    parser.add_argument(
        "--export-viewer-data",
        type=str,
        default="",
        help="Export a self-contained 3D viewer (viewer.html) and viewer_data.json to this folder or JSON path",
    )
    parser.add_argument(
        "--viewer-height-zero-mode",
        choices=["first-frame", "none"],
        default="first-frame",
        help="How to zero the viewer cylinder height. 'first-frame' subtracts the first valid cylinder height so the initial cylinder bottom aligns with the table surface; 'none' uses raw height (default: first-frame)",
    )
    parser.add_argument(
        "--viewer-height-offset-m",
        type=float,
        default=0.0,
        help="Additional manual viewer height offset after zeroing, in meters. Positive lifts the cylinder, negative lowers it (default: 0)",
    )

    parser.add_argument(
        "--quiet",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Hide verbose per-frame/per-circle console logs (default: on)",
    )
    parser.add_argument(
        "--progress-interval-sec",
        type=float,
        default=5.0,
        help="Print one progress update every N seconds while processing (default: 5)",
    )
    parser.add_argument(
        "--process-every-n-frames",
        type=int,
        default=1,
        help="Run AprilTag/pose processing every Nth frame (default: 1 = process all frames). Output timing is preserved.",
    )
    parser.add_argument(
        "--process-fps",
        type=float,
        default=0.0,
        help="Target processing FPS (e.g. 10). 0 disables this mode. Output timing is preserved.",
    )
    parser.add_argument(
        "--skipped-frame-output",
        choices=["hold-last-annotated", "raw-frame"],
        default="hold-last-annotated",
        help="Output behavior for skipped frames: hold-last-annotated or raw-frame (default: hold-last-annotated)",
    )

    args = parser.parse_args()

    # Merge CLI --circle args (if any) with the CIRCLES list at the top of the file
    all_circles: list[tuple[float, float, float, str]] = list(CIRCLES)

    # The old/manual 0 coordinate is unreliable in the current 4K dataset and
    # tends to duplicate/conflict with D. Skip it by default, but keep a flag
    # for backward compatibility.
    if not args.include_zero_circle:
        all_circles = [c for c in all_circles if str(c[3]) != "0"]

    if args.circle:
        base_count = len(all_circles)
        for i, c in enumerate(args.circle):
            all_circles.append((c[0], c[1], c[2], str(i + base_count + 1)))

    video_path = Path(args.video)
    if not video_path.exists():
        raise SystemExit(f"Video not found: {video_path}")

    K_raw, dist = build_camera_matrix()

    # pupil_apriltags expects grayscale images
    detector = Detector(
        families=args.family,
        nthreads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    # Separate detector for table corner tags (may use different decimate settings)
    table_detector = None
    if args.table_tag_ids:
        table_detector = Detector(
            families=args.family,
            nthreads=2,
            quad_decimate=float(args.table_quad_decimate),
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"Failed to open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames_raw = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    total_frames = total_frames_raw if total_frames_raw > 0 else None

    # Optional frame-range processing. Frame numbers are 1-based and inclusive.
    # Example: --start-frame 300 --end-frame 900 processes frames 300..900.
    start_frame = max(1, int(args.start_frame))
    end_frame = int(args.end_frame)
    if end_frame > 0 and end_frame < start_frame:
        raise SystemExit("--end-frame must be greater than or equal to --start-frame")
    if total_frames is not None and start_frame > total_frames:
        raise SystemExit(f"--start-frame {start_frame} is beyond the video length ({total_frames} frames)")
    if total_frames is not None and end_frame > total_frames:
        end_frame = total_frames

    if not cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame - 1) and start_frame > 1:
        raise SystemExit(f"Failed to seek to --start-frame {start_frame}")

    if end_frame > 0:
        selected_total_frames = end_frame - start_frame + 1
    elif total_frames is not None:
        selected_total_frames = total_frames - start_frame + 1
    else:
        selected_total_frames = None
    if args.max_frames > 0:
        selected_total_frames = min(selected_total_frames, int(args.max_frames)) if selected_total_frames is not None else int(args.max_frames)

    process_stride = max(1, int(args.process_every_n_frames))
    if float(args.process_fps) > 0.0 and float(fps) > 0.0:
        process_stride_from_fps = max(1, int(round(float(fps) / float(args.process_fps))))
        process_stride = max(process_stride, process_stride_from_fps)

    if not args.quiet:
        if start_frame > 1 or end_frame > 0 or args.max_frames > 0:
            selected_total_msg = selected_total_frames if selected_total_frames is not None else "unknown"
            end_frame_msg = end_frame if end_frame > 0 else "video-end"
            print(
                f"[range] start_frame={start_frame}, end_frame={end_frame_msg}, frames_to_write={selected_total_msg}",
                flush=True,
            )
        if process_stride > 1:
            effective_fps = (float(fps) / float(process_stride)) if float(fps) > 0.0 else 0.0
            print(
                f"[speed] Processing every {process_stride} frames (~{effective_fps:.2f} fps analyzed); output FPS/timing unchanged.",
                flush=True,
            )

    # --- CAMERA CALIBRATION / RESOLUTION FIX ---
    # The built-in K in this 4K file is already for 2160x3840 portrait video.
    # The old code scaled it as if it was 1080x1920, which double-scaled true
    # 4K videos and made the projected circles drift from the table circles.
    K, k_scale_msg = choose_camera_matrix_for_video(
        K_raw=K_raw,
        video_w=width,
        video_h=height,
        mode=args.camera_scale_mode,
        calib_w=int(args.calib_width),
        calib_h=int(args.calib_height),
    )
    if not args.quiet:
        print(f"[camera] video={width}x{height} | {k_scale_msg}", flush=True)
        print(
            f"[camera] K fx={K[0,0]:.3f}, fy={K[1,1]:.3f}, cx={K[0,2]:.3f}, cy={K[1,2]:.3f}",
            flush=True,
        )

    K_used = K
    dist_used = dist
    if args.undistort:
        K_used, _ = cv2.getOptimalNewCameraMatrix(K, dist, (width, height), 0.0, (width, height))
        dist_used = np.zeros((5,), dtype=np.float64)
        cam_params = (float(K_used[0, 0]), float(K_used[1, 1]), float(K_used[0, 2]), float(K_used[1, 2]))
    else:
        cam_params = (float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]))

    writer = None
    if args.annotated_out:
        out_path = Path(args.annotated_out)
        # Use mp4v to avoid codec issues on Windows; adjust if needed.
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # Compute output dimensions accounting for crop
        out_w, out_h = width, height
        if args.crop_top_percent > 0 or args.crop_bottom_percent > 0 or args.crop_left_percent > 0 or args.crop_right_percent > 0:
            if args.crop_resize:
                out_w, out_h = width, height
            else:
                out_h = height - int(height * args.crop_top_percent / 100.0) - int(height * args.crop_bottom_percent / 100.0)
                out_w = width - int(width * args.crop_left_percent / 100.0) - int(width * args.crop_right_percent / 100.0)
        writer = cv2.VideoWriter(str(out_path), fourcc, fps if fps > 0 else 30.0, (out_w, out_h))
        if not writer.isOpened():
            raise SystemExit(f"Failed to open VideoWriter: {out_path}")

    csv_file = None
    csv_writer = None
    if args.csv_out:
        csv_path = Path(args.csv_out)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "frame",
                "time_s",
                "tag_id",
                "reproj_rmse_px",
                "tvec_x_m",
                "tvec_y_m",
                "tvec_z_m",
                "yaw_z_deg",
                "pitch_y_deg",
                "roll_x_deg",
                "rvec_x",
                "rvec_y",
                "rvec_z",
            ]
        )

    frame_idx = start_frame - 1
    range_frame_idx = 0

    # --- Accumulators for freezing table / circles after N frames ---
    freeze_n = int(args.freeze_after) if args.freeze_after else 0
    # Table outline: collect outer-corner pixel positions per frame
    table_corners_accum: list[np.ndarray] = []  # each entry shape (M, 2)
    frozen_table_corners: np.ndarray | None = None
    # Circle reference pose: collect rvec/tvec per frame
    ref_rvec_accum: list[np.ndarray] = []
    ref_tvec_accum: list[np.ndarray] = []
    frozen_ref_rvec: np.ndarray | None = None
    frozen_ref_tvec: np.ndarray | None = None

    viewer_positions_local: list[list[float]] = []
    viewer_axes_local: list[list[float]] = []
    viewer_overlap_rows: list[list[float]] = []
    last_viewer_pos: list[float] | None = None
    last_viewer_axis: list[float] | None = None
    viewer_export_paths = _resolve_viewer_paths(args.export_viewer_data) if args.export_viewer_data else None
    viewer_height_baseline: float | None = None

    # Keep the last valid 2D cylinder footprint so temporary bad detections do not
    # shrink/explode the minimap or make the overlays jump randomly.
    last_valid_cylinder_ref_xy: tuple[float, float] | None = None
    # Option 3: keep the whole final cylinder state as the single source of truth.
    # Top view, overlap values, and viewer export all read from this same object.
    last_valid_cylinder_state: dict | None = None

    process_start_time = time.time()
    last_progress_print_time = process_start_time
    last_annotated_frame: np.ndarray | None = None

    def _apply_output_crop(frame: np.ndarray) -> np.ndarray:
        """Apply output crop/resize settings while preserving the configured output dimensions."""
        crop_top = args.crop_top_percent
        crop_bot = args.crop_bottom_percent
        crop_left = args.crop_left_percent
        crop_right = args.crop_right_percent

        out = frame
        if crop_top > 0 or crop_bot > 0 or crop_left > 0 or crop_right > 0:
            h_frame, w_frame = out.shape[:2]
            y1 = int(h_frame * crop_top / 100.0)
            y2 = h_frame - int(h_frame * crop_bot / 100.0)
            x1 = int(w_frame * crop_left / 100.0)
            x2 = w_frame - int(w_frame * crop_right / 100.0)
            out = out[y1:y2, x1:x2].copy()
            if args.crop_resize:
                out = cv2.resize(out, (w_frame, h_frame), interpolation=cv2.INTER_LINEAR)
        return out

    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                break

            frame_idx += 1
            if end_frame > 0 and frame_idx > end_frame:
                break

            range_frame_idx += 1
            if args.max_frames > 0 and range_frame_idx > int(args.max_frames):
                break

            t_s = (frame_idx - 1) / fps if fps and fps > 0 else 0.0

            now = time.time()
            interval_sec = max(0.0, float(args.progress_interval_sec))
            if interval_sec > 0 and (now - last_progress_print_time >= interval_sec):
                elapsed = max(now - process_start_time, 1e-9)
                processed_fps = range_frame_idx / elapsed
                if selected_total_frames is not None and selected_total_frames > 0:
                    remaining_frames = max(selected_total_frames - range_frame_idx, 0)
                    eta_sec = remaining_frames / processed_fps if processed_fps > 1e-9 else float("inf")
                    pct = 100.0 * range_frame_idx / selected_total_frames
                    print(
                        f"[progress] selected {range_frame_idx}/{selected_total_frames} frames ({pct:.1f}%) | "
                        f"video frame {frame_idx} | "
                        f"elapsed {format_seconds_hms(elapsed)} | "
                        f"speed {processed_fps:.2f} fps | ETA {format_seconds_hms(eta_sec)}",
                        flush=True,
                    )
                else:
                    print(
                        f"[progress] selected {range_frame_idx} frames processed | "
                        f"video frame {frame_idx} | "
                        f"elapsed {format_seconds_hms(elapsed)} | "
                        f"speed {processed_fps:.2f} fps",
                        flush=True,
                    )
                last_progress_print_time = now

            process_this_frame = ((range_frame_idx - 1) % process_stride == 0)

            # Fast path: skip expensive detection/pose work on selected frames while
            # still writing one output frame per input frame to keep duration unchanged.
            if not process_this_frame:
                if args.skipped_frame_output == "hold-last-annotated" and last_annotated_frame is not None:
                    display_frame = last_annotated_frame.copy()
                else:
                    display_frame = frame_bgr
                    if args.undistort:
                        display_frame = cv2.undistort(frame_bgr, K, dist, None, K_used)
                    display_frame = _apply_output_crop(display_frame)

                if csv_writer is not None:
                    csv_writer.writerow([frame_idx, f"{t_s:.6f}", "", "", "", "", "", "", "", "", "", "", ""])

                if viewer_export_paths is not None:
                    if last_viewer_pos is None:
                        last_viewer_pos = [0.0, 0.0, 0.0]
                    if last_viewer_axis is None:
                        last_viewer_axis = [0.0, 1.0, 0.0]
                    viewer_positions_local.append(list(last_viewer_pos))
                    viewer_axes_local.append(list(last_viewer_axis))
                    if viewer_overlap_rows:
                        viewer_overlap_rows.append(list(viewer_overlap_rows[-1]))
                    else:
                        viewer_overlap_rows.append([0.0 for _ in all_circles])

                if writer is not None:
                    writer.write(display_frame)

                if args.display:
                    cv2.imshow("AprilTag pose", display_frame)
                    key = cv2.waitKey(1)
                    if key == 27 or key == ord("q"):
                        break

                continue

            # Texts to overlay on the final display_frame (after crop)
            top_left_texts: list[tuple[str, tuple[int, int, int]]] = []
            bottom_left_texts: list[tuple[str, tuple[int, int, int]]] = []
            overlap_texts: list[tuple[str, tuple[int, int, int]]] = []

            # Cylinder tracking for 2D top-down minimap
            cyl_contact_cam = None
            current_cylinder_state = None
            final_cylinder_state = None
            draw_rvec = None
            draw_tvec = None

            # Deferred overlay so tracked tag / axes / cylinder can be drawn LAST
            # on top of the table outline, circles, and other tags.
            tracked_overlay = None

            frame_for_detection = frame_bgr
            if args.undistort:
                frame_for_detection = cv2.undistort(frame_bgr, K, dist, None, K_used)

            gray = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2GRAY)
            detect_kwargs = {"estimate_tag_pose": (args.pose_method == "detector")}
            if args.pose_method == "detector":
                detect_kwargs["camera_params"] = cam_params
                detect_kwargs["tag_size"] = float(args.tag_size_m)
            detections = detector.detect(gray, **detect_kwargs)

            tracked_margin = float(args.tracked_tag_min_decision_margin)

            candidates = []
            for d in detections:
                if int(d.tag_id) != int(args.tag_id):
                    continue
                if hasattr(d, "decision_margin") and float(d.decision_margin) < tracked_margin:
                    continue
                if hasattr(d, "hamming") and int(d.hamming) > int(args.max_hamming):
                    continue
                candidates.append(d)

            det = max(candidates, key=lambda x: float(getattr(x, "decision_margin", 0.0))) if candidates else None

            if det is not None:
                corners = np.asarray(det.corners, dtype=np.float64)

                # Always compute a PnP pose from corners so the tracked tag and the
                # table reference tag use the exact same coordinate convention.
                #
                # We may still use the detector pose for drawing if requested, but any
                # cross-tag geometry (cylinder/contact point mapped to Tag 1) should be
                # derived from the PnP pose to avoid frame-convention mismatches.
                pnp_rvec, pnp_tvec, pnp_rmse = estimate_pose_from_corners(
                    corners_xy=corners,
                    tag_size_m=args.tag_size_m,
                    K=K_used,
                    dist=dist_used,
                )

                detector_rvec = None
                detector_tvec = None
                detector_rmse = None
                if args.pose_method == "detector" and hasattr(det, "pose_R") and hasattr(det, "pose_t"):
                    detector_rvec, _ = cv2.Rodrigues(np.asarray(det.pose_R, dtype=np.float64))
                    detector_tvec = np.asarray(det.pose_t, dtype=np.float64).reshape(3, 1)
                    detector_rmse = reprojection_rmse_best_permutation_px(
                        corners_xy_px=corners,
                        tag_size_m=args.tag_size_m,
                        K=K_used,
                        dist=dist_used,
                        rvec=detector_rvec,
                        tvec=detector_tvec,
                    )

                if detector_rvec is not None and detector_tvec is not None:
                    if args.pose_method == "detector":
                        rvec, tvec, rmse = detector_rvec, detector_tvec, detector_rmse
                    else:
                        rvec, tvec, rmse = pnp_rvec, pnp_tvec, pnp_rmse
                else:
                    rvec, tvec, rmse = pnp_rvec, pnp_tvec, pnp_rmse

                yaw, pitch, roll = _rodrigues_to_euler_zyx_degrees(rvec)

                pose_ok = float(rmse) <= float(args.max_rmse_px)

                # CSV: always write pose if detected (even if pose_ok==False), so you can debug.
                if csv_writer is not None:
                    csv_writer.writerow(
                    [
                        frame_idx,
                        f"{t_s:.6f}",
                        int(det.tag_id),
                        f"{rmse:.4f}",
                        f"{tvec[0,0]:.6f}",
                        f"{tvec[1,0]:.6f}",
                        f"{tvec[2,0]:.6f}",
                        f"{yaw:.3f}",
                        f"{pitch:.3f}",
                        f"{roll:.3f}",
                        f"{rvec[0,0]:.8f}",
                        f"{rvec[1,0]:.8f}",
                        f"{rvec[2,0]:.8f}",
                    ]
                )

                if pose_ok:
                    # Compute effective base offsets (in tag frame)
                    cyl_ox = float(args.cylinder_offset_x_m)
                    cyl_oy = float(args.cylinder_offset_y_m)
                    cyl_oz = float(args.cylinder_offset_z_m)
                    cyl_d = float(args.cylinder_offset_along_axis_m)

                    axis_sign_override = None

                    # Anchor to the tag bottom edge center *in the image* (largest pixel Y), then apply a bias
                    # further away from tag center along that same tag-Y direction.
                    # Only defined for cylinder axis aligned with tag Y.
                    if args.cylinder_anchor == "tag-bottom" and args.cylinder_axis == "y":
                        half_tag = float(args.tag_size_m) / 2.0
                        bias = float(args.cylinder_bias_m)

                        # Determine which tag-Y direction points to the bottom edge in the image.
                        p_plus = np.array([[0.0, half_tag, 0.0]], dtype=np.float64).reshape(1, 1, 3)
                        p_minus = np.array([[0.0, -half_tag, 0.0]], dtype=np.float64).reshape(1, 1, 3)
                        y_plus = float(cv2.projectPoints(p_plus, rvec, tvec, K_used, dist_used)[0].reshape(2)[1])
                        y_minus = float(cv2.projectPoints(p_minus, rvec, tvec, K_used, dist_used)[0].reshape(2)[1])
                        sign_edge = 1 if y_plus >= y_minus else -1

                        # The tag-bottom line in tag coordinates is at y = sign_edge * half_tag.
                        desired_bottom_y = sign_edge * (half_tag + bias)

                        if args.cylinder_mode == "centered":
                            # bottom_y = cyl_oy + cyl_d - L/2
                            cyl_oy = desired_bottom_y + (float(args.cylinder_length_m) / 2.0) - cyl_d
                        else:
                            # from-base: base_y = cyl_oy + axis_sign * cyl_d
                            if args.cylinder_axis_sign == "+":
                                sign_for_base = 1
                            elif args.cylinder_axis_sign == "-":
                                sign_for_base = -1
                            else:
                                sign_for_base = sign_edge
                                axis_sign_override = sign_edge

                            cyl_oy = desired_bottom_y - (sign_for_base * cyl_d)

                    # Draw offset handling.
                    #
                    # draw_cyl_* is the cylinder position used for the rendered video outline.
                    # In option-2 mode (--sync-draw-offset-to-topview, default ON), the same
                    # adjusted position is also used for the top-view CYL marker, overlap
                    # percentages, and JSON/viewer export. This keeps the main video overlay,
                    # minimap, and 3D viewer synchronized.
                    #
                    # Disable with --no-sync-draw-offset-to-topview to restore the old behavior:
                    # the video outline is nudged visually, but the top-view/export uses the
                    # original geometric cyl_ox/cyl_oy/cyl_oz.
                    draw_cyl_ox = cyl_ox
                    draw_cyl_oy = cyl_oy + float(args.cylinder_draw_offset_y_m)
                    draw_cyl_oz = cyl_oz
                    use_draw_offset_for_geometry = bool(args.sync_draw_offset_to_topview)
                    geom_cyl_ox = draw_cyl_ox if use_draw_offset_for_geometry else cyl_ox
                    geom_cyl_oy = draw_cyl_oy if use_draw_offset_for_geometry else cyl_oy
                    geom_cyl_oz = draw_cyl_oz if use_draw_offset_for_geometry else cyl_oz

                    # Choose cylinder axis sign
                    if args.cylinder_axis_sign in {"+", "-"}:
                        axis_sign = 1 if args.cylinder_axis_sign == "+" else -1
                    elif axis_sign_override is not None:
                        axis_sign = int(axis_sign_override)
                    else:
                        axis_sign = choose_cylinder_axis_sign(
                            rvec=rvec,
                            tvec=tvec,
                            K=K_used,
                            dist=dist_used,
                            base_offset_xyz_m=(geom_cyl_ox, geom_cyl_oy, geom_cyl_oz),
                            offset_along_axis_m=cyl_d,
                            length_m=args.cylinder_length_m,
                            axis=args.cylinder_axis,
                            facing=args.cylinder_facing,
                        )

                    # Defer actual drawing until AFTER table/circles so this tracked
                    # tag overlay always appears on top.
                    tracked_overlay = {
                        "corners": corners.copy(),
                        "rvec": rvec.copy(),
                        "tvec": tvec.copy(),
                        "draw_offset_xyz": (draw_cyl_ox, draw_cyl_oy, draw_cyl_oz),
                        "offset_along_axis_m": cyl_d,
                        "length_m": float(args.cylinder_length_m),
                        "diameter_m": float(args.cylinder_diameter_m),
                        "axis": args.cylinder_axis,
                        "axis_sign": int(axis_sign),
                        "mode": args.cylinder_mode,
                        "tag_size_m": float(args.tag_size_m),
                    }

                    # Always use the tracked tag's PnP pose for any geometry that will
                    # later be expressed relative to Tag 1. This keeps both tags in the
                    # same pose convention even if the visual overlay is drawn using the
                    # detector pose.
                    cyl_rvec_map = pnp_rvec
                    cyl_tvec_map = pnp_tvec

                    # Option 3 / single-source-of-truth geometry:
                    # Compute the final footprint/contact point ONCE in the tracked-tag frame.
                    # The same state later drives the top-view marker, overlap percentages,
                    # and JSON/viewer export, avoiding separate calculations that can drift.
                    cyl_contact_tag = compute_cylinder_contact_point_tag(
                        offset_tag_xyz_m=(geom_cyl_ox, geom_cyl_oy, geom_cyl_oz),
                        offset_along_axis_m=cyl_d,
                        length_m=float(args.cylinder_length_m),
                        axis=args.cylinder_axis,
                        axis_sign=int(axis_sign),
                        mode=args.cylinder_mode,
                    )

                    T_cam_cyl = _rvec_tvec_to_T(cyl_rvec_map, cyl_tvec_map)
                    cyl_contact_cam = (T_cam_cyl[:3, :3] @ cyl_contact_tag) + T_cam_cyl[:3, 3]
                    current_cylinder_state = {
                        "contact_tag": tuple(float(v) for v in cyl_contact_tag),
                        "contact_cam": tuple(float(v) for v in cyl_contact_cam),
                        "radius_m": float(args.cylinder_diameter_m) / 2.0,
                        "axis_sign": int(axis_sign),
                        "axis": str(args.cylinder_axis),
                        "mode": str(args.cylinder_mode),
                    }

                    # No top-left pose text here; top-left is reserved for overlap percentages.
            else:
                if csv_writer is not None:
                    csv_writer.writerow([frame_idx, f"{t_s:.6f}", "", "", "", "", "", "", "", "", "", "", ""])

            # --- Table outline from corner tags (separate detection pipeline) ---
            if args.table_tag_ids and table_detector is not None:
                table_ids_set = set(args.table_tag_ids)
                # Run a completely independent detection for table tags
                table_detections_raw = table_detector.detect(gray)
                table_dets: dict[int, object] = {}
                for d in table_detections_raw:
                    tid = int(d.tag_id)
                    if tid not in table_ids_set:
                        continue
                    # Use table-specific (lenient) thresholds — not the tag-5 thresholds
                    if hasattr(d, "decision_margin") and float(d.decision_margin) < float(args.table_min_decision_margin):
                        continue
                    if hasattr(d, "hamming") and int(d.hamming) > int(args.table_max_hamming):
                        continue
                    # Keep best detection per tag id
                    if tid not in table_dets or float(getattr(d, "decision_margin", 0.0)) > float(
                        getattr(table_dets[tid], "decision_margin", 0.0)
                    ):
                        table_dets[tid] = d

                # In frame-skipping mode, we may not process exactly frame==freeze_n.
                # Freeze once we have passed freeze_n and have at least one sample.
                if freeze_n > 0 and range_frame_idx >= freeze_n and frozen_table_corners is None and len(table_corners_accum) > 0:
                    frozen_table_corners = np.mean(table_corners_accum, axis=0).astype(np.int32)
                    if not args.quiet:
                        print(f"[freeze] Table outline frozen from {len(table_corners_accum)} samples")

                if freeze_n > 0 and range_frame_idx >= freeze_n and frozen_ref_rvec is None and len(ref_rvec_accum) > 0:
                    frozen_ref_rvec = np.mean(ref_rvec_accum, axis=0)
                    frozen_ref_tvec = np.mean(ref_tvec_accum, axis=0)
                    if not args.quiet:
                        print(f"[freeze] Circle ref pose frozen from {len(ref_rvec_accum)} samples")

                # Decide whether we are still accumulating or already frozen
                use_frozen = freeze_n > 0 and range_frame_idx >= freeze_n and frozen_table_corners is not None

                # ---- Draw table outline ----
                if use_frozen:
                    # Draw from frozen averaged corners
                    poly = frozen_table_corners.reshape(-1, 1, 2)
                    cv2.polylines(frame_for_detection, [poly], isClosed=True,
                                  color=tuple(args.table_color), thickness=args.table_thickness)
                    for pt in frozen_table_corners:
                        cv2.circle(frame_for_detection, (int(pt[0]), int(pt[1])), 6, tuple(args.table_color), -1)
                    tbl_label = f"Table: frozen (avg of first {freeze_n} selected frames)"
                    bottom_left_texts.append((tbl_label, tuple(args.table_color)))
                elif len(table_dets) >= 2:
                    # Live drawing + accumulate corners for averaging
                    draw_table_outline(
                        frame_bgr=frame_for_detection,
                        table_detections=table_dets,
                        color=tuple(args.table_color),
                        thickness=args.table_thickness,
                    )
                    # Accumulate outer corners for freeze averaging
                    if freeze_n > 0 and range_frame_idx <= freeze_n:
                        centers = [np.asarray(d.center, dtype=np.float64) for d in table_dets.values()]
                        centroid = np.mean(centers, axis=0)
                        outer_corners = []
                        for d in table_dets.values():
                            corners = np.asarray(d.corners, dtype=np.float64).reshape(-1, 2)
                            dists = np.linalg.norm(corners - centroid, axis=1)
                            outer_corners.append(corners[int(np.argmax(dists))])
                        ordered = _order_corners_clockwise(outer_corners)
                        table_corners_accum.append(np.array(ordered, dtype=np.float64))

                    tbl_label = f"Table: {len(table_dets)}/{len(args.table_tag_ids)} corners detected (ids: {sorted(table_dets.keys())})"
                    bottom_left_texts.append((tbl_label, tuple(args.table_color)))

                # ---- Draw circles ----
                if all_circles and args.table_tag_size_m > 0:
                    ref_id = args.circle_ref_tag_id

                    if use_frozen and frozen_ref_rvec is not None:
                        # Use frozen averaged pose
                        draw_rvec, draw_tvec = frozen_ref_rvec, frozen_ref_tvec
                    elif ref_id in table_dets:
                        # Live pose + accumulate
                        ref_det = table_dets[ref_id]
                        ref_corners = np.asarray(ref_det.corners, dtype=np.float64)
                        try:
                            draw_rvec, draw_tvec, _ = estimate_pose_from_corners(
                                corners_xy=ref_corners,
                                tag_size_m=args.table_tag_size_m,
                                K=K_used,
                                dist=dist_used,
                            )
                            if freeze_n > 0 and range_frame_idx <= freeze_n:
                                ref_rvec_accum.append(draw_rvec.copy())
                                ref_tvec_accum.append(draw_tvec.copy())
                        except Exception as e:
                            if not args.quiet:
                                print(f"[circle] pose estimation failed for tag {ref_id}: {e}")
                            draw_rvec, draw_tvec = None, None
                    else:
                        if not args.quiet:
                            print(f"[circle] ref tag {ref_id} not detected (visible: {sorted(table_dets.keys())})")
                        draw_rvec, draw_tvec = None, None

                    if draw_rvec is not None and draw_tvec is not None:
                        if det is not None:
                            # Calculate Tag 5's position on the table surface by
                            # ray-casting its center PIXEL onto Tag 1's Z=0 plane.
                            # This avoids any frame-of-reference mismatch between
                            # Tag 5's pose (detector) and Tag 1's pose (PnP).
                            tag5_cx, tag5_cy = float(det.center[0]), float(det.center[1])
                            table_xy = pixel_to_table_surface(
                                tag5_cx, tag5_cy, K_used, draw_rvec, draw_tvec,
                            )
                            if table_xy is not None:
                                rx, ry = table_xy

                        for circ_idx, (cx_m, cy_m, cr_m, clabel) in enumerate(all_circles):
                            # Outer circle: double the area → radius × √2
                            outer_r = cr_m * math.sqrt(2.0)
                            draw_circle_on_table_surface(
                                frame_bgr=frame_for_detection,
                                K=K_used,
                                dist=dist_used,
                                rvec=draw_rvec,
                                tvec=draw_tvec,
                                x_offset_m=cx_m,
                                y_offset_m=cy_m,
                                radius_m=outer_r,
                                color=(0, 255, 0),
                                thickness=args.circle_thickness,
                            )
                            # Inner circle
                            draw_circle_on_table_surface(
                                frame_bgr=frame_for_detection,
                                K=K_used,
                                dist=dist_used,
                                rvec=draw_rvec,
                                tvec=draw_tvec,
                                x_offset_m=cx_m,
                                y_offset_m=cy_m,
                                radius_m=cr_m,
                                label=clabel,
                                color=tuple(args.circle_color),
                                thickness=args.circle_thickness,
                            )
                            if not args.quiet:
                                print(f"[circle {clabel}] drawn at ({cx_m}, {cy_m}) r={cr_m} outer_r={outer_r:.4f} from tag {ref_id}")

            # Draw the tracked AprilTag + axes + cylinder LAST so they appear above
            # the table outline, circles, and any other tag outlines.
            if tracked_overlay is not None:
                pts = tracked_overlay["corners"].reshape(-1, 2).astype(np.int32)
                cv2.polylines(frame_for_detection, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                try:
                    axis_len = tracked_overlay["tag_size_m"] * 0.5
                    cv2.drawFrameAxes(
                        frame_for_detection,
                        K_used,
                        dist_used,
                        tracked_overlay["rvec"],
                        tracked_overlay["tvec"],
                        axis_len,
                    )
                except Exception:
                    pass

                draw_cylinder_attached_to_tag(
                    frame_bgr=frame_for_detection,
                    K=K_used,
                    dist=dist_used,
                    rvec=tracked_overlay["rvec"],
                    tvec=tracked_overlay["tvec"],
                    offset_tag_xyz_m=tracked_overlay["draw_offset_xyz"],
                    offset_along_axis_m=tracked_overlay["offset_along_axis_m"],
                    length_m=tracked_overlay["length_m"],
                    diameter_m=tracked_overlay["diameter_m"],
                    axis=tracked_overlay["axis"],
                    axis_sign=tracked_overlay["axis_sign"],
                    mode=tracked_overlay["mode"],
                )

            # --- Crop the frame for display / output ---
            display_frame = _apply_output_crop(frame_for_detection)

            # --- Final cylinder state (Option 3 single source of truth) ---
            # Build this once, then reuse it for the top view, overlap percentages,
            # and exported viewer data. This prevents the video overlay, minimap,
            # and JSON simulation from using subtly different cylinder coordinates.
            cylinder_ref_xy = None
            cylinder_ref_xyz = None
            if cyl_contact_cam is not None:
                if draw_rvec is not None and draw_tvec is not None:
                    candidate_cylinder_state = build_final_cylinder_state(
                        cyl_contact_cam=cyl_contact_cam,
                        ref_rvec=draw_rvec,
                        ref_tvec=draw_tvec,
                        K=K_used,
                        dist=dist_used,
                        cylinder_radius_m=float(args.cylinder_diameter_m) / 2.0,
                    )

                    # If the current cylinder position is missing or obviously wrong,
                    # keep showing/exporting the last valid complete state until a good
                    # estimate comes back. The fallback is also shared by all outputs.
                    if candidate_cylinder_state is not None:
                        candidate_xy = candidate_cylinder_state.get("table_xy")
                        if is_reasonable_cylinder_position(candidate_xy, all_circles):
                            final_cylinder_state = candidate_cylinder_state
                            last_valid_cylinder_state = dict(candidate_cylinder_state)
                            last_valid_cylinder_ref_xy = candidate_xy
                        else:
                            final_cylinder_state = last_valid_cylinder_state
                    else:
                        final_cylinder_state = last_valid_cylinder_state
                else:
                    final_cylinder_state = last_valid_cylinder_state
                    if not args.quiet:
                        print("[minimap-warning] Tag 1 pose missing, cannot update final cylinder state this frame.")
            else:
                final_cylinder_state = last_valid_cylinder_state

            if final_cylinder_state is not None:
                cylinder_ref_xy = final_cylinder_state.get("table_xy")
                cylinder_ref_xyz = final_cylinder_state.get("ref_xyz")

            # --- 2D top-down minimap (top-right corner, drawn AFTER crop) ---
            if all_circles:
                # Draw minimap from the same final CYL XY used for overlap and export.
                draw_2d_topdown_view(
                    frame_bgr=display_frame,
                    circles=all_circles,
                    cylinder_pos_ref_xy=cylinder_ref_xy,
                    cylinder_radius_m=float(args.cylinder_diameter_m) / 2.0,
                    topview_rotation=int(args.topview_rotation),
                )

                # Compute top-left overlap percentages using the same final CYL XY.
                overlaps = compute_cylinder_circle_overlap_percentages(
                    circles=all_circles,
                    cylinder_pos_ref_xy=cylinder_ref_xy,
                    cylinder_radius_m=float(args.cylinder_diameter_m) / 2.0,
                )
                overlap_texts.append(("CYL overlap with table circles (%)", (255, 255, 255)))
                overlap_row_by_label = {str(label): float(pct) for label, pct in overlaps}
                viewer_overlap_rows.append([float(overlap_row_by_label.get(str(c[3]), 0.0)) for c in all_circles])
                overlaps = sorted(overlaps, key=lambda x: (-x[1], x[0]))
                for label, pct in overlaps:
                    color = (0, 0, 255) if pct > 0.0 else (200, 200, 200)
                    overlap_texts.append((f"{label}: {pct:.1f}%", color))
            elif viewer_export_paths is not None:
                viewer_overlap_rows.append([0.0 for _ in all_circles])

            # Draw top-left overlap labels
            current_y = 30
            for i, (text, color) in enumerate(overlap_texts):
                font_scale = 0.75 if i == 0 else 0.65
                thickness = 2 if i == 0 else 1
                cv2.putText(display_frame, text, (10, current_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv2.LINE_AA)
                current_y += 24 if i == 0 else 21

            # Draw bottom-left labels
            current_y = display_frame.shape[0] - 20
            for text, color in bottom_left_texts:
                cv2.putText(display_frame, text, (10, current_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
                current_y -= 30

            # --- Optional 3D viewer export data ---
            if viewer_export_paths is not None:
                viewer_pos = None
                viewer_axis = None

                if final_cylinder_state is not None:
                    # Map Tag1 frame into viewer frame: X->X, table height->Y, Y->Z.
                    # IMPORTANT: X/Z are taken from final_cylinder_state["table_xy"],
                    # the exact same XY used by the minimap and overlap calculation.
                    # ref_xyz is only used for the viewer height, so the 3D viewer
                    # cannot drift sideways from the top-view CYL marker.
                    state_xy = final_cylinder_state.get("table_xy")
                    state_xyz = final_cylinder_state.get("ref_xyz")
                    if state_xy is not None and state_xyz is not None:
                        raw_viewer_up = abs(float(state_xyz[2]))
                        if args.viewer_height_zero_mode == "first-frame":
                            if viewer_height_baseline is None:
                                viewer_height_baseline = raw_viewer_up
                            viewer_up = raw_viewer_up - viewer_height_baseline + float(args.viewer_height_offset_m)
                        else:
                            viewer_up = raw_viewer_up + float(args.viewer_height_offset_m)
                        viewer_up = max(0.0, viewer_up)
                        viewer_pos = [
                            float(state_xy[0]),
                            viewer_up,
                            float(state_xy[1]),
                        ]
                elif cylinder_ref_xy is not None:
                    viewer_pos = [float(cylinder_ref_xy[0]), 0.0, float(cylinder_ref_xy[1])]

                if tracked_overlay is not None and draw_rvec is not None:
                    try:
                        R_track, _ = cv2.Rodrigues(tracked_overlay["rvec"])
                        R_ref, _ = cv2.Rodrigues(draw_rvec)
                        sign = 1.0 if int(tracked_overlay["axis_sign"]) >= 0 else -1.0
                        axis_name = str(tracked_overlay["axis"]).lower()
                        if axis_name == "x":
                            axis_tag = np.array([sign, 0.0, 0.0], dtype=np.float64)
                        elif axis_name == "z":
                            axis_tag = np.array([0.0, 0.0, sign], dtype=np.float64)
                        else:
                            axis_tag = np.array([0.0, sign, 0.0], dtype=np.float64)
                        axis_cam = R_track @ axis_tag
                        axis_ref = (R_ref.T @ axis_cam).reshape(3)
                        nrm = float(np.linalg.norm(axis_ref))
                        if nrm > 1e-9:
                            axis_ref = axis_ref / nrm
                            viewer_axis = [float(axis_ref[0]), float(axis_ref[2]), float(axis_ref[1])]
                    except Exception:
                        viewer_axis = None

                if viewer_pos is None:
                    viewer_pos = last_viewer_pos
                if viewer_axis is None:
                    viewer_axis = last_viewer_axis
                if viewer_pos is None:
                    viewer_pos = [0.0, 0.0, 0.0]
                if viewer_axis is None:
                    viewer_axis = [0.0, 1.0, 0.0]

                last_viewer_pos = list(viewer_pos)
                last_viewer_axis = list(viewer_axis)
                viewer_positions_local.append(list(viewer_pos))
                viewer_axes_local.append(list(viewer_axis))

            last_annotated_frame = display_frame.copy()

            if writer is not None:
                writer.write(display_frame)

            if args.display:
                cv2.imshow("AprilTag pose", display_frame)
                key = cv2.waitKey(1)
                if key == 27 or key == ord("q"):
                    break

    finally:
        if viewer_export_paths is not None:
            try:
                json_path, html_path = viewer_export_paths
                Path(json_path).parent.mkdir(parents=True, exist_ok=True)
                xs = [float(c[0]) for c in all_circles] if all_circles else [0.0]
                zs = [float(c[1]) for c in all_circles] if all_circles else [0.0]
                rs = [float(c[2]) for c in all_circles] if all_circles else [0.05]
                pad = max(rs) * 2.0 if rs else 0.1
                table_width = max(0.2, (max(xs) - min(xs)) + 2.0 * pad)
                table_depth = max(0.2, (max(zs) - min(zs)) + 2.0 * pad)
                center_x = 0.5 * (min(xs) + max(xs))
                center_z = 0.5 * (min(zs) + max(zs))

                circle_centers_local = [[float(c[0]) - center_x, float(c[1]) - center_z] for c in all_circles]
                shifted_positions = [
                    [float(p[0]) - center_x, float(p[1]), float(p[2]) - center_z]
                    for p in viewer_positions_local
                ]

                viewer_data = {
                    "fps": float(fps if fps and fps > 0 else 30.0),
                    "table": {"width": float(table_width), "depth": float(table_depth), "height": 0.04},
                    "circles": {
                        "centers_local": circle_centers_local,
                        "labels": [str(c[3]) for c in all_circles],
                        "radius": float(max(rs) if rs else 0.045),
                        "outer_radius": float((max(rs) if rs else 0.045) * math.sqrt(2.0)),
                    },
                    "cylinder": {
                        "radius": float(args.cylinder_diameter_m) / 2.0,
                        "height": float(args.cylinder_length_m),
                    },
                    "positions_local": shifted_positions,
                    "axes_local": viewer_axes_local,
                    "overlap_rows": viewer_overlap_rows,
                    "position_source": "option3_single_final_cylinder_state",
                }

                Path(json_path).write_text(
                    json.dumps(_sanitize_for_json(viewer_data), indent=2, allow_nan=False),
                    encoding="utf-8",
                )
                _write_embedded_viewer_html(html_path, viewer_data)
                if not args.quiet:
                    print(f"[viewer] wrote {html_path} and {json_path}")
            except Exception as exc:
                print(f"[viewer-warning] failed to write viewer export: {exc}")

        if csv_file is not None:
            csv_file.close()
        cap.release()
        if writer is not None:
            writer.release()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
