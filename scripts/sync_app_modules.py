"""레포의 공용 모듈을 App Lab 앱 폴더로 복사한다.

App Lab은 앱 폴더(예: apps/qrover_drive)만 실행 환경에 넣기 때문에,
레포 최상위의 beacon/, state_machine/, vision/ 같은 모듈을 앱 안에서 바로 import하지 못할 수 있다 (#1).
그래서 앱을 실행하기 전에 필요한 파일을 앱의 python/ 폴더로 복사한다.
복사본은 .gitignore로 커밋에서 빠진다. 고칠 때는 항상 **원본**을 고치고 다시 실행한다.

실행 (레포 최상위 폴더에서):
    python scripts/sync_app_modules.py
"""

from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]

# (앱 python 폴더, 복사할 원본 파일들) — 앱이 늘어나면 여기에 추가
APPS = {
    "apps/qrover_drive/python": [
        "beacon/__init__.py",
        "beacon/peak_detector.py",
        "state_machine/__init__.py",
        "state_machine/beacon_search.py",
        "state_machine/front_parking.py",
        "vision/__init__.py",
        "vision/aruco.py",
    ],
}


def main() -> None:
    for app_dir, files in APPS.items():
        for rel in files:
            src = ROOT / rel
            dst = ROOT / app_dir / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            print(f"복사: {rel} → {app_dir}/{rel}")


if __name__ == "__main__":
    main()
