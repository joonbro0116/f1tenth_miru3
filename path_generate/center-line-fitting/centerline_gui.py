#!/usr/bin/env python3
"""
Centerline Fitting GUI 런처
사용자 친화적인 인터페이스로 centerline fitting 파라미터를 설정하고 실행할 수 있습니다.
"""

import sys
import os

# src 디렉토리를 Python path에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, 'src')
if src_dir not in sys.path:
    sys.path.append(src_dir)

try:
    from src.parameter_gui import main
    
    if __name__ == "__main__":
        print("Centerline Fitting GUI를 시작합니다...")
        print("GUI 창에서 다음 작업을 수행할 수 있습니다:")
        print("- PGM/YAML 파일 선택")
        print("- 디노이징 파라미터 조정")
        print("- 스무딩 파라미터 조정")
        print("- 처리 결과 시각화")
        print("- 설정 저장/로드")
        print()
        
        main()
        
except ImportError as e:
    print(f"GUI 모듈을 불러올 수 없습니다: {e}")
    print("필요한 패키지를 설치해주세요:")
    print("pip install tkinter matplotlib numpy scikit-learn scikit-image scipy")
    sys.exit(1)
except Exception as e:
    print(f"GUI 실행 중 오류가 발생했습니다: {e}")
    sys.exit(1)