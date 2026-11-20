"""비전 패키지.

ArUco 영상 인식에만 OpenCV가 필요하다. 순수 정렬 계산/단위 테스트에서는
cv2 없이도 ``vision.align`` 과 ``vision.aruco_auto_align`` 을 가져올 수 있게 한다.
"""

__all__ = ["ArucoDetector", "Marker", "detect_markers"]


def __getattr__(name):
    if name in __all__:
        from importlib import import_module

        return getattr(import_module(".aruco", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
