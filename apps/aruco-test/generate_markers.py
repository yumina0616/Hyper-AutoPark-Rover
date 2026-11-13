from pathlib import Path
import cv2

# TODO: 실험 및 #6 마커 규격 이슈에서 최종 결정
DICT_TYPE = cv2.aruco.DICT_4X4_50
MARKER_IDS = [0, 1, 2, 3, 4]
MARKER_SIZE_PX = 400
BORDER_PX = 60  # 인식률을 높이기 위한 흰색 여백


def main():
    out_dir = Path(__file__).parent / "markers"
    out_dir.mkdir(exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)

    for marker_id in MARKER_IDS:
        if hasattr(cv2.aruco, "generateImageMarker"):
            img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE_PX)
        else:
            img = cv2.aruco.drawMarker(aruco_dict, marker_id, MARKER_SIZE_PX)

        img_with_border = cv2.copyMakeBorder(
            img,
            BORDER_PX,
            BORDER_PX,
            BORDER_PX,
            BORDER_PX,
            cv2.BORDER_CONSTANT,
            value=255,
        )

        out_path = out_dir / f"marker_{marker_id}.png"
        cv2.imwrite(str(out_path), img_with_border)
        print(f"생성 완료: {out_path}")


if __name__ == "__main__":
    main()
