#!/usr/bin/env python3
"""
Interactive chessboard camera calibration.

This is the engine both usb_cam/calibrate.sh and usb_cam/calibrate_iphone.sh
call as:

    calibrate_camera.py --device SRC --cols N --rows M --board-units auto
        --square 0.20 --model rational [--cam-fps 0] [--auto-capture 1.5]
        [--require-coverage] --out OUT.npz --save-views DIR
        [--load-views DIR]

Controls while the live window is focused:
    SPACE   capture the current frame as a calibration view (only works
            while a chessboard is detected)
    c       run calibration on every captured view so far and save --out
            (also writes the queued view PNGs if --save-views was given)
    q       quit without calibrating; anything captured but not saved with
            'c' is discarded

Board sizing (--cols/--rows/--board-units):
    Boards are commonly described by how many SQUARES they have per side,
    but OpenCV's detector wants the number of INNER corners (one less than
    the square count per side). --board-units lets you say which one you
    are giving it:
        squares   --cols/--rows are square counts; corners = cols-1, rows-1
        corners   --cols/--rows are already inner-corner counts
        auto      try the squares interpretation first, then the corners
                  interpretation, and lock onto whichever one the detector
                  actually finds on the physical board in frame

Distortion model (--model):
    standard    the default 5-coefficient OpenCV model (k1,k2,p1,p2,k3)
    rational    the 8-coefficient rational model (adds k4,k5,k6). Needed for
                wide-FOV lenses (~90deg+): the 5-coefficient radial
                polynomial can fold before it reaches the frame edge, which
                silently produces wrong poses out there while the center
                still looks fine.

Coverage (--require-coverage):
    The live overlay divides the frame into a 6x4 grid of 24 cells. A cell
    turns from red to green once a captured view's chessboard corners
    overlapped it. With --require-coverage, pressing 'c' refuses to
    calibrate until every cell is green AND at least one captured view put
    a corner past 90% of the image's corner radius (i.e. genuinely near an
    edge/corner of the frame, not just centered shots taken from different
    angles). This is what catches a calibration that will fold at the edges
    before it gets baked into --out.
"""
import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO_ROOT))

# Reuse the repo's own device-opening logic (AVFoundation on macOS, V4L2 on
# Linux, avf:<id> identities, http(s)/rtsp streams) so this script opens a
# camera exactly the same way every other tool in usb_cam/ does. Falls back
# to a plain cv2.VideoCapture if that module isn't importable (e.g. this
# script is copied out on its own), so it still runs standalone.
try:
    from usb_cam.track_rov_topview import (
        open_capture as _repo_open_capture,
        resolve_capture_device as _repo_resolve_capture_device,
    )
    _HAVE_REPO_CAPTURE = True
except ImportError:
    _HAVE_REPO_CAPTURE = False

    def _repo_resolve_capture_device(device):
        text = str(device)
        return int(text) if text.isdigit() else text

    def _repo_open_capture(device, width=None, height=None, fps=None):
        text = str(device)
        if text.startswith(("http://", "rtsp://")):
            cap = cv2.VideoCapture(text)
        else:
            cap = cv2.VideoCapture(_repo_resolve_capture_device(device))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            cap.set(cv2.CAP_PROP_FPS, float(fps))
        return cap


GRID_COLS = 6
GRID_ROWS = 4
EDGE_RADIUS_TARGET = 0.90  # --require-coverage also wants >=90% corner radius
STABLE_MOVE_PX = 6.0       # centroid jitter tolerance for --auto-capture


# --------------------------------------------------------------------------
# Board pattern handling
# --------------------------------------------------------------------------

def candidate_patterns(cols, rows, board_units):
    """Inner-corner (w, h) candidates to try, in priority order."""
    squares_pattern = (cols - 1, rows - 1)
    corners_pattern = (cols, rows)
    if board_units == "squares":
        return [squares_pattern]
    if board_units == "corners":
        return [corners_pattern]
    return [squares_pattern, corners_pattern]  # auto


def find_corners(gray, pattern):
    """Try to locate the chessboard for one candidate pattern."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if not found:
        return False, None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
    return True, corners


def detect_board(gray, patterns, locked_pattern):
    """Detect the board, preferring an already-locked pattern.

    Returns (found, corners, pattern_used). Once a pattern has produced a
    detection it is tried first on every later frame so --board-units auto
    doesn't keep re-guessing every frame.
    """
    ordered = patterns
    if locked_pattern is not None:
        ordered = [locked_pattern] + [p for p in patterns if p != locked_pattern]
    for pattern in ordered:
        found, corners = find_corners(gray, pattern)
        if found:
            return True, corners, pattern
    return False, None, None


def object_points_for(pattern, square_m):
    w, h = pattern
    objp = np.zeros((w * h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2) * square_m
    return objp


# --------------------------------------------------------------------------
# Coverage grid
# --------------------------------------------------------------------------

class CoverageGrid:
    """Tracks which of a 6x4 grid of frame cells have seen a captured view."""

    def __init__(self, cols=GRID_COLS, rows=GRID_ROWS):
        self.cols = cols
        self.rows = rows
        self.covered = np.zeros((rows, cols), dtype=bool)
        self.max_radius_frac = 0.0

    def mark(self, corners, frame_shape):
        h, w = frame_shape[:2]
        pts = corners.reshape(-1, 2)
        xs, ys = pts[:, 0], pts[:, 1]
        x0, x1 = max(xs.min(), 0), min(xs.max(), w - 1)
        y0, y1 = max(ys.min(), 0), min(ys.max(), h - 1)
        c0, c1 = int(x0 / w * self.cols), int(x1 / w * self.cols)
        r0, r1 = int(y0 / h * self.rows), int(y1 / h * self.rows)
        c1 = min(c1, self.cols - 1)
        r1 = min(r1, self.rows - 1)
        self.covered[r0:r1 + 1, c0:c1 + 1] = True

        cx, cy = w / 2.0, h / 2.0
        half_diag = (cx ** 2 + cy ** 2) ** 0.5
        dists = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
        frac = float(dists.max() / half_diag) if half_diag else 0.0
        self.max_radius_frac = max(self.max_radius_frac, frac)

    def all_covered(self):
        return bool(self.covered.all())

    def cell_count(self):
        return int(self.covered.sum()), self.covered.size

    def draw(self, frame):
        h, w = frame.shape[:2]
        cw, ch = w / self.cols, h / self.rows
        overlay = frame.copy()
        for r in range(self.rows):
            for c in range(self.cols):
                color = (0, 160, 0) if self.covered[r, c] else (0, 0, 200)
                x0, y0 = int(c * cw), int(r * ch)
                x1, y1 = int((c + 1) * cw), int((r + 1) * ch)
                cv2.rectangle(overlay, (x0, y0), (x1, y1), color, 2)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, dst=frame)
        return frame


# --------------------------------------------------------------------------
# View capture / saving
# --------------------------------------------------------------------------

def save_view(save_dir, index, frame, corners, pattern):
    save_dir.mkdir(parents=True, exist_ok=True)
    raw_path = save_dir / f"view_{index:02d}.png"
    corners_path = save_dir / f"view_{index:02d}_corners.png"
    cv2.imwrite(str(raw_path), frame)
    annotated = frame.copy()
    cv2.drawChessboardCorners(annotated, pattern, corners, True)
    cv2.imwrite(str(corners_path), annotated)


def load_views(load_dir, patterns, square_m):
    """Re-detect corners in previously saved view_NN.png files."""
    obj_points, img_points, image_size = [], [], None
    locked_pattern = None
    view_files = sorted(
        p for p in Path(load_dir).glob("view_*.png")
        if not p.stem.endswith("_corners")
    )
    for path in view_files:
        img = cv2.imread(str(path))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        found, corners, pattern = detect_board(gray, patterns, locked_pattern)
        if not found:
            print(f"[load-views] skipping {path.name}: board not found")
            continue
        locked_pattern = pattern
        obj_points.append(object_points_for(pattern, square_m))
        img_points.append(corners)
    print(f"[load-views] loaded {len(obj_points)} usable view(s) from {load_dir}")
    return obj_points, img_points, image_size, locked_pattern


# --------------------------------------------------------------------------
# Calibration
# --------------------------------------------------------------------------

def run_calibration(obj_points, img_points, image_size, model):
    flags = cv2.CALIB_RATIONAL_MODEL if model == "rational" else 0
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None, flags=flags)
    return ret, mtx, dist, rvecs, tvecs


def save_calibration(out_path, mtx, dist, image_size, ret, model, pattern, square_m):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        out_path,
        camera_matrix=mtx,
        dist_coeffs=dist,
        image_width=image_size[0],
        image_height=image_size[1],
        reprojection_error=ret,
        model=model,
        pattern_cols=pattern[0],
        pattern_rows=pattern[1],
        square_size_m=square_m,
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
    )
    print(f"[saved] {out_path}  (reprojection RMS error: {ret:.4f} px)")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", required=True,
                     help="camera source: /dev/videoN, avf:<id>, bare index, "
                          "or an http(s)/rtsp URL")
    ap.add_argument("--cols", type=int, required=True,
                     help="board width, in squares or inner corners "
                          "(see --board-units)")
    ap.add_argument("--rows", type=int, required=True,
                     help="board height, in squares or inner corners")
    ap.add_argument("--board-units", choices=["squares", "corners", "auto"],
                     default="auto",
                     help="how to interpret --cols/--rows (default: auto)")
    ap.add_argument("--square", type=float, required=True,
                     help="square edge length in meters")
    ap.add_argument("--model", choices=["standard", "rational"],
                     default="standard",
                     help="distortion model (default: standard)")
    ap.add_argument("--cam-fps", type=float, default=None,
                     help="request this capture FPS; 0 or omitted skips "
                          "the request entirely (network/loopback sources "
                          "set their own rate)")
    ap.add_argument("--width", type=int, default=None,
                     help="request this capture width (optional)")
    ap.add_argument("--height", type=int, default=None,
                     help="request this capture height (optional)")
    ap.add_argument("--auto-capture", type=float, default=None,
                     metavar="SECONDS",
                     help="auto-capture a view once the board has been "
                          "held roughly still for this many seconds "
                          "(SPACE still works too)")
    ap.add_argument("--require-coverage", action="store_true",
                     help="refuse to calibrate (on 'c') until all 24 grid "
                          f"cells are covered and >= {int(EDGE_RADIUS_TARGET*100)}%% "
                          "of the corner radius has been reached")
    ap.add_argument("--out", required=True, help="output .npz path")
    ap.add_argument("--save-views", default=None, metavar="DIR",
                     help="write view_NN.png + view_NN_corners.png here "
                          "as each view is captured")
    ap.add_argument("--load-views", default=None, metavar="DIR",
                     help="seed calibration with previously saved "
                          "view_NN.png files from this directory before "
                          "the live loop starts")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    patterns = candidate_patterns(args.cols, args.rows, args.board_units)
    square_m = args.square

    obj_points, img_points = [], []
    image_size = None
    locked_pattern = None

    if args.load_views:
        obj_points, img_points, image_size, locked_pattern = load_views(
            args.load_views, patterns, square_m)

    fps = args.cam_fps if args.cam_fps else None
    cap = _repo_open_capture(args.device, width=args.width,
                              height=args.height, fps=fps)
    if not cap or not cap.isOpened():
        sys.exit(f"error: could not open camera source {args.device!r}")

    coverage = CoverageGrid()
    save_dir = Path(args.save_views) if args.save_views else None
    view_index = len(obj_points)

    stable_since = None
    last_centroid = None
    window = "calibrate_camera"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    print("[controls] SPACE=capture  c=calibrate+save  q=quit")
    print(f"[board] trying pattern(s): {patterns} "
          f"(--board-units {args.board_units})")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[warn] no frame from camera source; retrying...")
                time.sleep(0.1)
                continue
            if image_size is None:
                image_size = (frame.shape[1], frame.shape[0])

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners, pattern = detect_board(gray, patterns, locked_pattern)

            display = frame.copy()
            coverage.draw(display)

            do_auto_capture = False
            if found:
                locked_pattern = pattern
                cv2.drawChessboardCorners(display, pattern, corners, True)
                centroid = corners.reshape(-1, 2).mean(axis=0)
                if args.auto_capture:
                    if (last_centroid is not None and
                            np.linalg.norm(centroid - last_centroid) < STABLE_MOVE_PX):
                        if stable_since is None:
                            stable_since = time.time()
                        elif time.time() - stable_since >= args.auto_capture:
                            do_auto_capture = True
                            stable_since = None
                    else:
                        stable_since = time.time()
                    last_centroid = centroid
            else:
                stable_since = None
                last_centroid = None

            covered_n, covered_total = coverage.cell_count()
            status = (f"views:{len(obj_points)}  "
                      f"coverage:{covered_n}/{covered_total}  "
                      f"radius:{coverage.max_radius_frac*100:.0f}%  "
                      f"board:{'FOUND' if found else 'searching...'}")
            cv2.putText(display, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(window, display)

            key = cv2.waitKey(1) & 0xFF
            want_capture = do_auto_capture or key == ord(' ')

            if want_capture and found:
                obj_points.append(object_points_for(pattern, square_m))
                img_points.append(corners)
                coverage.mark(corners, frame.shape)
                if save_dir is not None:
                    save_view(save_dir, view_index, frame, corners, pattern)
                view_index += 1
                print(f"[capture] view {view_index} "
                      f"(pattern {pattern[0]}x{pattern[1]} inner corners)")
            elif want_capture and not found:
                print("[capture] ignored: board not detected in this frame")

            if key == ord('c'):
                if not obj_points:
                    print("[calibrate] no views captured yet")
                    continue
                if args.require_coverage:
                    missing = []
                    if not coverage.all_covered():
                        n, total = coverage.cell_count()
                        missing.append(f"only {n}/{total} grid cells covered")
                    if coverage.max_radius_frac < EDGE_RADIUS_TARGET:
                        missing.append(
                            f"max corner radius reached is only "
                            f"{coverage.max_radius_frac*100:.0f}%, need "
                            f"{int(EDGE_RADIUS_TARGET*100)}%")
                    if missing:
                        print("[calibrate] refused (--require-coverage): "
                              + "; ".join(missing))
                        continue
                print(f"[calibrate] running on {len(obj_points)} view(s)...")
                ret, mtx, dist, _, _ = run_calibration(
                    obj_points, img_points, image_size, args.model)
                save_calibration(args.out, mtx, dist, image_size, ret,
                                  args.model, locked_pattern, square_m)
                break

            if key == ord('q') or key == 27:
                print("[quit] discarding session, nothing saved")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()