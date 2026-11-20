"""rx_scan.py 출력을 받아서 최댓값 감지 결과를 보여주는 도구 (로버를 움직이지 않음)

rx_scan.py(#21) 출력 형식: `2026-11-16T18:30:01.125, 1, -58`  (받은 시각, 칸 번호, RSSI)

사용법 (레포 최상위 폴더에서)
    # 1) 실시간: UNO Q에서 수신하면서 로버를 손으로 밀어 비콘 앞을 지나가 보기
    python3 beacon/rx_scan.py | python3 -m beacon.peak_monitor

    # 2) 녹화 재생: 저장해 둔 출력을 넣어서 파라미터 맞춰 보기
    python3 beacon/rx_scan.py > run1.csv        (UNO Q에서 녹화)
    python -m beacon.peak_monitor run1.csv --drop-db 3 --confirm 0.4

"지나쳤다"가 확정되면, 그때까지 걸린 시간과 "같은 속도로 몇 초 되돌아가면 되는지"를 출력한다.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from typing import Iterable, Optional, Tuple

from beacon.peak_detector import PeakConfig, PeakDetector


def parse_line(line: str) -> Optional[Tuple[float, int, float]]:
    """`ISO시각, 칸번호, RSSI` 한 줄 → (초, 칸, rssi). 형식이 아니면 None (안내 문구 등)."""
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 3:
        return None
    try:
        t = datetime.fromisoformat(parts[0]).timestamp()
        return t, int(parts[1]), float(parts[2])
    except ValueError:
        return None


def run(lines: Iterable[str], detector: PeakDetector, slot: Optional[int]) -> None:
    t0 = None
    for line in lines:
        parsed = parse_line(line)
        if parsed is None:
            continue
        t, slot_id, rssi = parsed
        if slot is not None and slot_id != slot:
            continue
        t0 = t if t0 is None else t0

        just_passed = detector.update(t, rssi)
        filt = "  ---" if detector.filtered is None else f"{detector.filtered:6.1f}"
        peak = "  ---" if detector.peak_rssi is None else f"{detector.peak_rssi:6.1f}"
        print(f"{t - t0:7.2f}s  rssi={rssi:5.0f}  평균={filt}  최대={peak}", flush=True)

        if just_passed:
            peak_t = detector.peak_time()
            print("\n=== 지나쳤다! ===", flush=True)
            print(f"최댓값 시각: {peak_t - t0:.2f}s (평균 {detector.peak_rssi:.1f} dBm)", flush=True)
            print(f"판단 시각:   {t - t0:.2f}s", flush=True)
            print(f"→ 같은 속도로 약 {t - peak_t:.2f}초 후진하면 최댓값 지점\n", flush=True)


def main() -> None:
    p = argparse.ArgumentParser(description="비콘 RSSI 최댓값 감지 확인 (로버를 움직이지 않음)")
    p.add_argument("file", nargs="?", help="rx_scan.py 출력 파일. 없으면 표준 입력(파이프)")
    p.add_argument("--slot", type=int, help="이 칸 번호만 사용")
    p.add_argument("--window", type=float, default=PeakConfig.window_sec)
    p.add_argument("--drop-db", type=float, default=PeakConfig.drop_db)
    p.add_argument("--confirm", type=float, default=PeakConfig.confirm_sec)
    p.add_argument("--min-peak", type=float, default=PeakConfig.min_peak_rssi)
    args = p.parse_args()

    det = PeakDetector(PeakConfig(window_sec=args.window, drop_db=args.drop_db,
                                  confirm_sec=args.confirm, min_peak_rssi=args.min_peak))
    if args.file:
        with open(args.file, encoding="utf-8") as f:
            run(f, det, args.slot)
    else:
        run(sys.stdin, det, args.slot)


if __name__ == "__main__":
    main()
