"""ArUco 마커로 로버의 자세(벽까지 거리·옆 오차·틀어진 각도)를 추정한다. (실험 단계)

err_x는 "마커가 화면 가운데에 있나"만 알려준다. 로버가 칸 정면에서 옆으로 벗어나
비스듬히 마커를 보고 있어도 err_x는 0이 될 수 있다. 이 모듈은 마커의 네 꼭짓점과
실제 마커 크기로 solvePnP를 풀어서, 칸(마커가 붙은 벽) 기준 자세를 계산한다.

좌표 약속 (로버가 벽을 바라보는 상황 기준)
    wall_dist_mm  카메라에서 벽(마커 평면)까지 수직 거리
    offset_mm     칸 중심선(마커 중심을 지나는 벽의 수직선)에서 카메라가 옆으로 벗어난 거리
                  + = 카메라가 중심선보다 오른쪽, - = 왼쪽
    yaw_deg       로버 정면과 칸 방향(벽에 수직, 벽 쪽)의 각도 차이
                  + = 칸 방향이 로버 정면보다 오른쪽 → 오른쪽으로 돌아야 똑바름
                  - = 왼쪽으로 돌아야 똑바름
    bearing_deg   로버 정면 기준 마커 중심의 방향 (+ = 오른쪽). err_x를 각도로 바꾼 값

주의
    - 카메라 내부값(초점거리)을 모르면 화각(hfov_deg)으로 대략 계산한다. 정확도를 올리려면
      체스판으로 카메라 보정을 해서 camera_matrix·dist_coeffs를 넣는다.
    - 카메라가 로버 중심에 있다고 가정한다. 옆으로 붙어 있으면 그 거리만큼 offset_mm을 보정해야 한다.
    - 거의 정면(0° 근처)에서는 평면 마커 특성상 yaw가 ±몇 도씩 흔들릴 수 있다. 실측으로 확인할 것.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

# 인쇄한 마커의 검은 부분 한 변 (흰 테두리 제외). 10cm(#6)에서 5cm로 변경 (2026-11-24)
DEFAULT_MARKER_MM = 50.0
# TODO: 카메라 보정으로 교체. Logitech C270 사양 대각 화각 55° → 4:3 화면 가로 약 45°
DEFAULT_HFOV_DEG = 45.0
# 이보다 재투영 오차가 크면 자세를 믿지 않는다 (꼭짓점이 잘못 잡힌 경우 등)
MAX_REPROJ_PX = 5.0


@dataclass(frozen=True)
class Pose:
    wall_dist_mm: float
    offset_mm: float
    yaw_deg: float
    bearing_deg: float


def camera_matrix_from_fov(width: int, height: int, hfov_deg: float = DEFAULT_HFOV_DEG):
    """화각으로 대략적인 카메라 행렬을 만든다 (렌즈 왜곡 없음 가정)."""
    import numpy as np

    if width <= 0 or height <= 0 or not 0 < hfov_deg < 180:
        raise ValueError("해상도는 양수, 화각은 0~180도여야 합니다")
    f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])


def marker_object_points(marker_mm: float):
    """마커 꼭짓점의 실제 좌표 (마커 중심 원점, x 오른쪽, y 위, z 마커 앞쪽).

    순서는 vision.aruco.Marker.corners와 같다: 좌상 → 우상 → 우하 → 좌하.
    """
    import numpy as np

    h = marker_mm / 2.0
    return np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64
    )


def pose_from_rt(rvec, tvec) -> Pose:
    """solvePnP 결과(마커 → 카메라 좌표 변환)를 벽 기준 자세로 바꾼다."""
    import cv2
    import numpy as np

    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    t = np.asarray(tvec, dtype=np.float64).reshape(3)

    # 카메라 위치를 마커(벽) 좌표로: x = 벽을 따라 오른쪽, z = 벽에서 앞으로
    cam_in_marker = -rot.T @ t
    offset_mm = float(cam_in_marker[0])
    wall_dist_mm = float(cam_in_marker[2])

    # 칸 방향(벽 쪽, 마커 -z)을 카메라 좌표로: x 오른쪽, z 정면
    into_wall = rot @ np.array([0.0, 0.0, -1.0])
    yaw_deg = math.degrees(math.atan2(into_wall[0], into_wall[2]))
    bearing_deg = math.degrees(math.atan2(t[0], t[2]))
    return Pose(wall_dist_mm, offset_mm, yaw_deg, bearing_deg)


def estimate_pose(
    corners: Sequence[Sequence[float]],
    camera_matrix,
    dist_coeffs=None,
    marker_mm: float = DEFAULT_MARKER_MM,
) -> Optional[Pose]:
    """마커 꼭짓점 4개로 자세를 추정한다. 계산할 수 없으면 None."""
    import cv2
    import numpy as np

    if corners is None or len(corners) != 4 or marker_mm <= 0:
        return None
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    obj = marker_object_points(marker_mm)
    dist = np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)

    # 평면 마커는 해가 두 개 나올 수 있고, OpenCV 4.14의 IPPE는 정면에 가까운 장면에서
    # 재투영 오차가 큰 잘못된 해를 내기도 했다. 후보를 모아 재투영 오차가 가장 작은 해를 쓴다.
    candidates = []
    if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
        try:
            _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj, img, camera_matrix, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            candidates += list(zip(rvecs, tvecs))
        except cv2.error:
            pass
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if ok:
        candidates.append((rvec, tvec))

    best: Optional[tuple[float, Pose]] = None
    for rvec, tvec in candidates:
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist)
        err = float(np.abs(projected.reshape(4, 2) - img).max())
        pose = pose_from_rt(rvec, tvec)
        if pose.wall_dist_mm <= 0:  # 마커 뒤쪽 해: 물리적으로 불가능
            continue
        if best is None or err < best[0]:
            best = (err, pose)
    if best is None or best[0] > MAX_REPROJ_PX:
        return None
    return best[1]


class PoseAverager:
    """최근 n개 자세의 평균 (값이 흔들릴 때 읽기 쉽게)."""

    def __init__(self, n: int = 10) -> None:
        if n < 1:
            raise ValueError("n은 1 이상이어야 합니다")
        self.n = n
        self._items: list[Pose] = []

    def add(self, pose: Pose) -> Pose:
        self._items.append(pose)
        if len(self._items) > self.n:
            self._items.pop(0)
        k = len(self._items)
        return Pose(
            sum(p.wall_dist_mm for p in self._items) / k,
            sum(p.offset_mm for p in self._items) / k,
            sum(p.yaw_deg for p in self._items) / k,
            sum(p.bearing_deg for p in self._items) / k,
        )

    def clear(self) -> None:
        self._items.clear()

    def __len__(self) -> int:
        return len(self._items)


class PoseEstimator:
    """해상도별 카메라 행렬을 기억해 두고 마커마다 자세를 계산한다."""

    def __init__(
        self,
        marker_mm: float = DEFAULT_MARKER_MM,
        hfov_deg: float = DEFAULT_HFOV_DEG,
        camera_matrix=None,
        dist_coeffs=None,
    ) -> None:
        self.marker_mm = marker_mm
        self.hfov_deg = hfov_deg
        self._fixed_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self._cache: dict[tuple[int, int], object] = {}

    def matrix_for(self, width: int, height: int):
        if self._fixed_matrix is not None:
            return self._fixed_matrix
        key = (width, height)
        if key not in self._cache:
            self._cache[key] = camera_matrix_from_fov(width, height, self.hfov_deg)
        return self._cache[key]

    def estimate(self, marker: object, width: int, height: int) -> Optional[Pose]:
        corners = getattr(marker, "corners", None)
        if not corners:
            return None
        return estimate_pose(corners, self.matrix_for(width, height), self.dist_coeffs,
                             self.marker_mm)
