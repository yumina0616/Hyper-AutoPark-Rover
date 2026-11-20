"""UNO Q App Lab Linux (MPU) - Logitech C270 / ArUco front parking.

Change TARGET_MARKER_ID and calibrate thresholds in sketch.ino.
This program needs OpenCV with the aruco module.
"""
import time
import cv2
import numpy as np
from arduino.app_utils import App, Bridge

TARGET_MARKER_ID = 0           # parking bay 0..4
CAMERA_INDEX = 0               # change if C270 isn't /dev/video0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SHOW_PREVIEW = False           # set True only when a GUI display is available
CALL_TIMEOUT_S = 0.35

aruco = cv2.aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
if hasattr(aruco, 'ArucoDetector'):
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
    def detect_markers(gray):
        return detector.detectMarkers(gray)
else:
    parameters = aruco.DetectorParameters_create()
    def detect_markers(gray):
        return aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

camera = None
last_log_time = 0.0
last_state = None
terminal = False
STATE_NAMES = {
    0: 'WAIT', 1: 'ALIGN', 2: 'APPROACH', 3: 'PARKED',
    4: 'SAFETY_STOP', 5: 'MARKER_LOST', 6: 'SENSOR_ERROR',
}


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError('Cannot open Logitech C270 camera: check CAMERA_INDEX and USB')
    return cap


def get_target_marker(frame):
    """Return (side_px, offset_px, corners) or (-1, 0, None)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners_list, ids, _ = detect_markers(gray)
    if ids is None:
        return -1, 0, None
    for corners, detected_id in zip(corners_list, ids.flatten()):
        if int(detected_id) != TARGET_MARKER_ID:
            continue
        pts = corners.reshape((4, 2))
        # Mean side length in pixels, more robust than bounding box width.
        side_lengths = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
        side_px = int(round(float(np.mean(side_lengths))))
        center_x = float(np.mean(pts[:, 0]))
        offset_px = int(round(center_x - frame.shape[1] / 2))
        return side_px, offset_px, pts
    return -1, 0, None


def main_loop():
    global camera, last_log_time, last_state, terminal
    if terminal:
        time.sleep(0.5)
        return
    if camera is None:
        camera = open_camera()
        print(f'Camera opened. TARGET_MARKER_ID={TARGET_MARKER_ID}')

    ok, frame = camera.read()
    if not ok:
        # No new vision data -> MCU watchdog brakes after 600ms.
        raise RuntimeError('Camera frame read failed; MCU watchdog will stop motors')

    side_px, offset_px, corners = get_target_marker(frame)
    marker_id = TARGET_MARKER_ID if side_px >= 0 else -1

    # RPC once per frame; Arduino owns all safety checks and motor commands.
    state = int(Bridge.call('set_marker', marker_id, max(0, side_px), offset_px,
                            timeout=CALL_TIMEOUT_S))

    now = time.monotonic()
    if state != last_state or now - last_log_time > 1.0:
        print(f'state={STATE_NAMES.get(state, state)}, '
              f'marker_id={marker_id}, side_px={side_px}, offset_px={offset_px}')
        last_state = state
        last_log_time = now

    if SHOW_PREVIEW:
        if corners is not None:
            cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.line(frame, (frame.shape[1] // 2, 0),
                 (frame.shape[1] // 2, frame.shape[0]), (255, 0, 0), 1)
        cv2.putText(frame, STATE_NAMES.get(state, str(state)), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow('Front Parking C270', frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            raise KeyboardInterrupt

    if state in (3, 4, 6):
        print(f'Terminal state {STATE_NAMES[state]}; Arduino holds brake. '
              'Reset MCU after correcting the cause to restart.')
        terminal = True
        camera.release()
        camera = None
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        # App Lab expects the user's loop to keep running; avoid excessive RPC.
        time.sleep(0.5)


if __name__ == '__main__':
    App.run(user_loop=main_loop)
