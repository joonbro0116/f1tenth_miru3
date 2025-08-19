from PIL import Image
import numpy as np
from collections import Counter

# PGM 파일 경로
pgm_file = "/home/f1/f1tenth_ws/maps/gap_map_final.pgm"

# 이미지 읽기
img = Image.open(pgm_file)
img_np = np.array(img)

# 픽셀값 평탄화
pixels = img_np.flatten()

# 픽셀값별 개수 계산
pixel_counts = Counter(pixels)

# 결과 출력 (픽셀값 순서대로)
for val in sorted(pixel_counts.keys()):
    print(f"픽셀값 {val}: {pixel_counts[val]}개")
