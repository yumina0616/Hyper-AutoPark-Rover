"""로버 카메라 화면을 노트북 브라우저로 보는 도구 (ArUco 인식 확인용).

UNO Q에서 실행하면 카메라 영상 위에 인식 결과와 숫자를 그려서 웹페이지로 내보낸다.
노트북 브라우저에서 http://hyperq2.local:8080 을 열면 보인다.
모터에는 아무 명령도 보내지 않는다 (정렬 단계의 바퀴 값은 '제안값'만 표시).

실행 (UNO Q에 SSH 접속 후, 레포 최상위 폴더에서):
    python3 -m vision.camera_view --camera 2          # UNO Q의 C270은 /dev/video2, 목표 마커 0번
    python3 -m vision.camera_view --camera 2 --target 3
    python3 -m vision.camera_view --camera 2 --hfov 45 --marker-mm 50
    (노트북 웹캠은 --camera 0)

화면에 보이는 것
    - 초록 테두리: 목표 마커 / 노란 테두리: 다른 마커
    - 가운데 세로선과 양옆 점선: 화면 중앙과 정렬 허용 범위(align_tol)
    - 마커마다: ID, err_x(좌우 오차 -1~1), size(한 변 픽셀), skew(기울어짐)
    - FPS: 카메라 읽기 + 검출까지 포함한 초당 처리 장수
    - 정렬 단계(vision.aruco_auto_align): 상태와 바퀴 제안값 L/R
    - 목표 마커 자세(vision.pose, 실험 단계): 방금 값(now)과 최근 10프레임 평균(avg)
        dist   벽(마커 평면)까지 수직 거리 mm
        offset 칸 중심선에서 옆으로 벗어난 거리 mm (+ = 카메라가 오른쪽)
        yaw    칸 방향과 로버 정면의 각도 차 (+ = 오른쪽으로 돌아야 똑바름)
      화각(--hfov)으로 계산한 대략값이다. 카메라 보정 전에는 dist가 몇 % 틀릴 수 있다.
    - tilt: 평균 yaw로 기울기 판정. ±--tilt-tol(기본 3°) 안이면 STRAIGHT(똑바름),
      넘으면 어느 쪽으로 돌아야 똑바른지 (turn RIGHT / turn LEFT)

브라우저 버튼
    - 사진 저장: 지금 화면을 보드의 --snap-dir 폴더에 저장 (기본 ~/qrover_snaps)
    - 정렬 초기화: ALIGNED 등으로 끝난 정렬 단계를 처음부터 다시 시작

종료: Ctrl+C
사진은 저장소에 커밋하지 않는다 (AGENTS.md 7장). 필요하면 공유 드라이브에 올린다.
"""

from __future__ import annotations

import argparse
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional, Sequence

from vision.align import marker_skew
from vision.aruco_auto_align import ArucoAlignStage, MarkerFrame, StageConfig, StageResult
from vision.pose import DEFAULT_HFOV_DEG, DEFAULT_MARKER_MM, Pose


# ---------------------------------------------------------------------------
# 순수 계산 (카메라·OpenCV 없이 테스트 가능)
# ---------------------------------------------------------------------------


class FpsMeter:
    """최근 window_sec 동안 처리한 프레임 수로 FPS를 계산한다."""

    def __init__(self, window_sec: float = 1.0) -> None:
        self.window_sec = window_sec
        self._times: deque[float] = deque()

    def tick(self, now: float) -> None:
        self._times.append(now)
        while self._times and now - self._times[0] > self.window_sec:
            self._times.popleft()

    @property
    def fps(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 0 else 0.0


def marker_line(marker: object) -> str:
    """마커 하나의 숫자 요약 (화면·콘솔 공용, 영어만: OpenCV가 한글을 못 그림)."""
    skew = marker_skew(marker)
    skew_txt = "n/a" if skew is None else f"{skew:+.3f}"
    return (
        f"ID {marker.id}  err_x {marker.err_x:+.3f}  "
        f"size {marker.size_px:.1f}px  skew {skew_txt}"
    )


def stage_line(result: StageResult) -> str:
    """정렬 단계 결과 요약."""
    text = f"align stage: {result.state}  L={result.left:+d} R={result.right:+d}"
    if result.err_x is not None:
        text += f"  err {result.err_x:+.3f}"
    return text


def pose_line(label: str, pose: Optional[Pose]) -> str:
    """목표 마커 자세 한 줄."""
    if pose is None:
        return f"pose {label}: n/a"
    return (
        f"pose {label}: dist {pose.wall_dist_mm:5.0f}mm  offset {pose.offset_mm:+5.0f}mm  "
        f"yaw {pose.yaw_deg:+5.1f}deg"
    )


# TODO: 실험으로 결정. 칸 안에서 벽에 닿지 않는 최대 각도 (로버 폭·길이로 계산)
DEFAULT_TILT_TOL_DEG = 3.0


def tilt_line(pose: Optional[Pose], tol_deg: float = DEFAULT_TILT_TOL_DEG) -> str:
    """로버가 칸(벽)에 대해 기울어졌는지 한눈에 보이게 판정한다 (평균 자세 사용 권장)."""
    if pose is None:
        return "tilt: n/a (no target marker)"
    yaw = pose.yaw_deg
    if abs(yaw) <= tol_deg:
        return f"tilt: STRAIGHT  ({yaw:+.1f}deg, tol +-{tol_deg:g})"
    side = "RIGHT" if yaw > 0 else "LEFT"
    return f"tilt: {yaw:+.1f}deg -> turn {side} to straighten (tol +-{tol_deg:g})"


def status_lines(
    fps: float,
    detect_ms: float,
    width: int,
    height: int,
    target_id: int,
    markers: Sequence[object],
    stage: StageResult,
    pose_now: Optional[Pose] = None,
    pose_avg: Optional[Pose] = None,
    tilt_tol_deg: float = DEFAULT_TILT_TOL_DEG,
) -> list[str]:
    """화면 위에 그릴 줄 목록. 목표 마커를 맨 위에 둔다."""
    lines = [
        f"FPS {fps:4.1f}  detect {detect_ms:5.1f}ms  {width}x{height}  target ID {target_id}",
        stage_line(stage),
        pose_line("now", pose_now),
        pose_line("avg", pose_avg),
        tilt_line(pose_avg, tilt_tol_deg),
    ]
    ordered = sorted(markers, key=lambda m: (m.id != target_id, m.id))
    if ordered:
        lines += [marker_line(m) for m in ordered]
    else:
        lines.append("no marker")
    return lines


# ---------------------------------------------------------------------------
# 카메라 → 검출 → 그림 → JPEG
# ---------------------------------------------------------------------------


@dataclass
class ViewConfig:
    camera: int = 0
    width: int = 640
    height: int = 480
    target_id: int = 0
    jpeg_quality: int = 70
    snap_dir: Path = Path.home() / "qrover_snaps"
    marker_mm: float = DEFAULT_MARKER_MM
    hfov_deg: float = DEFAULT_HFOV_DEG
    tilt_tol_deg: float = DEFAULT_TILT_TOL_DEG


class CameraView:
    """백그라운드 스레드에서 카메라를 읽고, 그림을 그린 최신 JPEG를 보관한다."""

    def __init__(self, cfg: ViewConfig) -> None:
        self.cfg = cfg
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._frame = None  # 그림을 그린 최신 프레임 (사진 저장용)
        self._seq = 0
        self._error: Optional[str] = None
        self._stop = threading.Event()
        self._reset_requested = threading.Event()
        # 화면 확인용이라 제한 시간으로 끝나지 않게 길게 둔다
        self.stage = ArucoAlignStage(cfg.target_id, StageConfig(timeout_sec=24 * 3600))
        from vision.pose import PoseAverager, PoseEstimator

        self.pose_estimator = PoseEstimator(marker_mm=cfg.marker_mm, hfov_deg=cfg.hfov_deg)
        self.pose_avg = PoseAverager(10)
        self._thread = threading.Thread(target=self._loop, daemon=True, name="camera-view")

    # --- 외부(웹 서버)에서 쓰는 함수 -------------------------------------------

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=1.0)

    @property
    def error(self) -> Optional[str]:
        return self._error

    def wait_jpeg(self, last_seq: int, timeout: float = 2.0) -> tuple[int, Optional[bytes]]:
        """last_seq보다 새 프레임이 나올 때까지 기다렸다가 (seq, jpeg)를 돌려준다."""
        with self._cond:
            self._cond.wait_for(
                lambda: (self._jpeg is not None and self._seq != last_seq)
                or self._stop.is_set()
                or self._error is not None,
                timeout=timeout,
            )
            return self._seq, self._jpeg

    def request_reset(self) -> None:
        self._reset_requested.set()

    def save_snapshot(self) -> Optional[Path]:
        import cv2

        with self._cond:
            frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            return None
        self.cfg.snap_dir.mkdir(parents=True, exist_ok=True)
        path = self.cfg.snap_dir / time.strftime("snap_%Y%m%d_%H%M%S.jpg")
        cv2.imwrite(str(path), frame)
        return path

    # --- 카메라 스레드 ----------------------------------------------------------

    def _open_camera(self):
        import sys

        import cv2

        # Linux(UNO Q)에서는 V4L2를 직접 지정해야 설정이 잘 먹는다
        if sys.platform.startswith("linux"):
            cap = cv2.VideoCapture(self.cfg.camera, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(self.cfg.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"카메라 {self.cfg.camera}번을 열 수 없음 (ls /dev/video* 확인)")
        return cap

    def _loop(self) -> None:
        import cv2

        from vision.aruco import ArucoDetector

        cap = None
        try:
            cap = self._open_camera()
            detector = ArucoDetector()
            meter = FpsMeter()
            last_print = 0.0
            self.stage.reset(time.time())
            while not self._stop.is_set():
                if self._reset_requested.is_set():
                    self._reset_requested.clear()
                    self.stage.reset(time.time())
                    self.pose_avg.clear()
                    print("[camera_view] 정렬 단계·자세 평균 초기화", flush=True)

                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError("카메라 프레임 읽기 실패")
                captured_t = time.time()

                t0 = time.perf_counter()
                markers = detector.detect_markers(frame)
                detect_ms = (time.perf_counter() - t0) * 1000.0
                meter.tick(time.time())

                seq = self._seq + 1
                result = self.stage.step(MarkerFrame(seq, captured_t, markers), now=time.time())
                h, w = frame.shape[:2]
                target = next((m for m in markers if m.id == self.cfg.target_id), None)
                pose_now = None if target is None else self.pose_estimator.estimate(target, w, h)
                if pose_now is None:
                    self.pose_avg.clear()  # 끊기면 평균도 새로 시작
                    pose_avg = None
                else:
                    pose_avg = self.pose_avg.add(pose_now)
                lines = status_lines(meter.fps, detect_ms, w, h, self.cfg.target_id, markers,
                                     result, pose_now, pose_avg, self.cfg.tilt_tol_deg)
                self._draw(cv2, frame, markers, lines)

                ok, buf = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.cfg.jpeg_quality]
                )
                if ok:
                    with self._cond:
                        self._jpeg = buf.tobytes()
                        self._frame = frame
                        self._seq = seq
                        self._cond.notify_all()

                now = time.time()
                if now - last_print >= 1.0:  # SSH 콘솔에도 1초마다 요약
                    print(" | ".join(lines), flush=True)
                    last_print = now
        except Exception as exc:  # 카메라 문제는 화면·콘솔에 알리고 멈춘다
            self._error = f"{type(exc).__name__}: {exc}"
            print(f"[camera_view] 오류: {self._error}", flush=True)
            with self._cond:
                self._cond.notify_all()
        finally:
            if cap is not None:
                cap.release()

    def _draw(self, cv2, frame, markers, lines) -> None:
        h, w = frame.shape[:2]
        cx = w // 2
        tol_px = int(self.stage.aligner.cfg.align_tol * (w / 2))

        # 화면 중앙선과 정렬 허용 범위
        cv2.line(frame, (cx, 0), (cx, h), (255, 0, 255), 1)  # 분홍: 어떤 배경에서도 보이게
        for x in (cx - tol_px, cx + tol_px):
            for y in range(0, h, 12):
                cv2.line(frame, (x, y), (x, min(y + 6, h)), (255, 0, 255), 1)

        # 마커 테두리: 목표는 초록, 나머지는 노랑
        for m in markers:
            if len(m.corners) != 4:
                continue
            color = (0, 220, 0) if m.id == self.cfg.target_id else (0, 220, 220)
            pts = [(int(x), int(y)) for x, y in m.corners]
            for i in range(4):
                cv2.line(frame, pts[i], pts[(i + 1) % 4], color, 2)
            cv2.circle(frame, pts[0], 4, (0, 0, 255), -1)  # 좌상 꼭짓점 (마커 방향 확인용)
            mx = int(sum(p[0] for p in pts) / 4)
            my = int(sum(p[1] for p in pts) / 4)
            cv2.putText(frame, str(m.id), (mx - 8, my + 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, color, 2)

        # 숫자 표시 (검은 배경 위 흰 글씨)
        for i, text in enumerate(lines):
            y = 22 + i * 22
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(frame, (6, y - th - 5), (12 + tw, y + 5), (0, 0, 0), -1)
            cv2.putText(frame, text, (9, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1)


# ---------------------------------------------------------------------------
# 웹 서버
# ---------------------------------------------------------------------------

PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Q-ROVER camera</title>
<style>
 body {{ background:#111; color:#eee; font-family:sans-serif; margin:16px; }}
 img {{ max-width:100%; border:1px solid #444; }}
 button {{ font-size:16px; padding:8px 14px; margin:8px 8px 8px 0; }}
 #msg {{ color:#8f8; }}
</style></head>
<body>
<h3>Q-ROVER 카메라 (목표 마커 {target})</h3>
<img src="/stream" alt="camera stream">
<div>
 <button onclick="act('/snap')">사진 저장</button>
 <button onclick="act('/reset')">정렬·평균 초기화</button>
 <span id="msg"></span>
</div>
<script>
async function act(path) {{
  const r = await fetch(path, {{method: 'POST'}});
  document.getElementById('msg').textContent = await r.text();
}}
</script>
</body></html>
"""

BOUNDARY = "qroverframe"


def make_handler(view: CameraView):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # 요청마다 찍히는 로그는 끈다
            pass

        def _text(self, code: int, text: str) -> None:
            body = text.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                body = PAGE.format(target=view.cfg.target_id).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self._stream()
            else:
                self._text(404, "없는 주소")

        def do_POST(self):
            if self.path == "/snap":
                path = view.save_snapshot()
                self._text(200, f"저장: {path}" if path else "아직 프레임 없음")
            elif self.path == "/reset":
                view.request_reset()
                self._text(200, "정렬 단계 초기화")
            else:
                self._text(404, "없는 주소")

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = -1
            try:
                while True:
                    if view.error:
                        break
                    new_seq, jpeg = view.wait_jpeg(seq)
                    if jpeg is None or new_seq == seq:
                        continue
                    seq = new_seq
                    self.wfile.write(
                        f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # 브라우저 탭을 닫음

    return Handler


def main() -> None:
    p = argparse.ArgumentParser(description="로버 카메라 화면을 브라우저로 보기 (ArUco 확인)")
    p.add_argument("--camera", type=int, default=0,
                   help="카메라 번호 (UNO Q의 C270은 2 = /dev/video2, 노트북 웹캠은 보통 0)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--target", type=int, default=0, help="목표 마커 번호 0~4 (기본 0)")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--host", default="0.0.0.0", help="기본: 같은 네트워크의 다른 기기에서 접속 가능")
    p.add_argument("--quality", type=int, default=70, help="JPEG 화질 1~100 (낮을수록 빠름)")
    p.add_argument("--snap-dir", type=Path, default=Path.home() / "qrover_snaps")
    p.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_MM,
                   help="마커 검은 부분 한 변 mm (흰 테두리 제외, 기본 50)")
    p.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG,
                   help="카메라 가로 화각(도). 카메라 보정 전 대략값 (기본 45)")
    p.add_argument("--tilt-tol", type=float, default=DEFAULT_TILT_TOL_DEG,
                   help="이 각도(도) 안이면 똑바름으로 표시 (기본 3)")
    args = p.parse_args()

    cfg = ViewConfig(args.camera, args.width, args.height, args.target, args.quality, args.snap_dir,
                     args.marker_mm, args.hfov, args.tilt_tol)
    view = CameraView(cfg)
    view.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(view))
    server.daemon_threads = True
    print(f"[camera_view] 브라우저에서 http://<보드 이름 또는 IP>:{args.port} 열기", flush=True)
    print("[camera_view] 예: http://hyperq2.local:%d  (종료: Ctrl+C)" % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[camera_view] 종료", flush=True)
    finally:
        server.server_close()
        view.stop()


if __name__ == "__main__":
    main()
