"""UNO Q iBeacon receiver for ESP32 DevKitC V1 transmitters.

Requires: sudo apt install python3-bleak
Run on UNO Q: python3 ~/rx_scan.py
Output: t_recv, slot_id, rssi
"""

import asyncio
from datetime import datetime

from bleak import BleakScanner


# Must exactly match the UUID in esp32_tx.ino.
BEACON_UUID = "7A1B2C3D-4E5F-6071-8293-A4B5C6D7E8F9"
APPLE_COMPANY_ID = 0x004C
IBEACON_PREFIX = b"\x02\x15"
EXPECTED_MINOR = 0  # Team convention


def format_uuid(data: bytes) -> str:
    """Format 16 UUID bytes in standard 8-4-4-4-12 notation."""
    return (
        f"{data[0:4].hex()}-"
        f"{data[4:6].hex()}-"
        f"{data[6:8].hex()}-"
        f"{data[8:10].hex()}-"
        f"{data[10:16].hex()}"
    ).upper()


def detection_callback(device, advertisement_data):
    # Bleak strips the 2-byte company ID from the dictionary value.
    data = advertisement_data.manufacturer_data.get(APPLE_COMPANY_ID)
    if data is None or len(data) != 23:
        return

    # iBeacon format: prefix(2) + UUID(16) + Major(2) + Minor(2) + power(1)
    if data[:2] != IBEACON_PREFIX:
        return

    received_uuid = format_uuid(data[2:18])
    if received_uuid != BEACON_UUID:
        return

    slot_id = int.from_bytes(data[18:20], "big")
    minor = int.from_bytes(data[20:22], "big")

    # Accept any valid slot (0-4), but enforce the project Minor convention.
    if not (0 <= slot_id <= 4) or minor != EXPECTED_MINOR:
        return

    # 보드와 App Lab 컨테이너의 로컬 시간대가 달라도 같은 epoch로 해석되도록
    # UTC 오프셋(+09:00 등)을 출력에 포함한다.
    t_recv = datetime.now().astimezone().isoformat(timespec="milliseconds")
    rssi = advertisement_data.rssi
    print(f"{t_recv}, {slot_id}, {rssi}", flush=True)


async def main():
    print("Starting BLE iBeacon Scanner...")
    print(f"Target UUID: {BEACON_UUID}")
    print(f"Expected Minor: {EXPECTED_MINOR}")
    print("Duplicate advertisements: ENABLED")
    print("Output: t_recv, slot_id, rssi")
    print("Press Ctrl+C to stop.\n")

    scanner = BleakScanner(
        detection_callback=detection_callback,
        scanning_mode="active",
        bluez={"filters": {"DuplicateData": True}},
    )

    await scanner.start()
    try:
        while True:
            await asyncio.sleep(1)
    finally:
        await scanner.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nBLE Scanner stopped.")
