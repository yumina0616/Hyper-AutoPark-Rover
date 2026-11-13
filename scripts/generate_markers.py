import cv2
from pathlib import Path

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

output_dir = Path("markers")
output_dir.mkdir(exist_ok=True)

for marker_id in range(5):

    marker = cv2.aruco.generateImageMarker(
        dictionary,
        marker_id,
        400
    )

    marker = cv2.copyMakeBorder(
        marker, 60, 60, 60, 60,
        cv2.BORDER_CONSTANT,
        value=255
    )
  
    filename = output_dir / f"marker_{marker_id}.png"

    cv2.imwrite(str(filename), marker)

    print(f"Created: {filename}")
