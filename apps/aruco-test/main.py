from pathlib import Path
import sys
import cv2

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from vision.aruco import detect_markers


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("에러: 카메라를 열 수 없습니다. 맥북 카메라 권한을 확인해 주세요.")
        return

    print("카메라 테스트 시작! 종료하려면 영상 창을 클릭한 상태에서 'q' 키를 누르세요.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽지 못했습니다.")
            break

        markers = detect_markers(frame)
        height, width = frame.shape[:2]

        # 화면 정중앙 기준선 표시 (err_x = 0 위치)
        cv2.line(frame, (width // 2, 0), (width // 2, height), (200, 200, 200), 1)

        if markers:
            for idx, m in enumerate(markers):
                msg = f"ID={m.id} | err_x={m.err_x:+.3f} | size_px={m.size_px:.1f}"
                print(msg)
                cv2.putText(
                    frame,
                    msg,
                    (20, 40 + idx * 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
        else:
            cv2.putText(
                frame,
                "No marker detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        cv2.imshow("ArUco Test (Press 'q' to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
