import tkinter as tk
from tkinter import ttk, filedialog, messagebox, Toplevel
import json
import os
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

class ParameterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Centerline Fitting 파라미터 설정")
        self.root.geometry("600x800")
        
        # 파라미터 기본값
        self.params = {
            # 파일 경로
            'pgm_path': '',
            'yaml_path': '',
            'output_dir': '/home/f1/f1tenth_ws/joon_path_generate/final_centerline/output/',
            
            # 디노이징 파라미터
            'denoise_radius': 2.0,
            'denoise_min_samples_ratio': 8.0,
            'denoise_min_cluster_ratio': 0.01,
            'denoise_min_cluster_min': 10,
            
            # 스무딩 파라미터
            'smoothing_method': 'curvature',
            'curvature_threshold': 0.3,
            'gaussian_sigma': 2.0,
            'spline_points_ratio': 4.0,  # 2배 더 촘촘하게
            'smoothing_window_size': 60,
            'smoothing_iterations': 5,
            'smoothing_blend_factor': 0.3,
            
            # 기본 처리 파라미터
            'subsample_period': 1,
            'plot_mode': 1,
            'calculate_track_widths': False
        }
        
        self.create_widgets()
        self.load_default_params()
        
        # 결과 파일 저장
        self.last_result_file = None
        self.last_pgm_file = None
        
    def create_slider_with_entry(self, parent, row, label, variable, from_, to_, 
                                resolution=0.1, width=200, entry_width=10):
        """슬라이더와 직접 입력 칸을 결합한 위젯 생성"""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
        
        # 슬라이더
        scale = ttk.Scale(parent, from_=from_, to=to_, variable=variable, 
                         orient=tk.HORIZONTAL, length=width)
        scale.grid(row=row, column=1, padx=5)
        
        # 직접 입력 칸
        entry = ttk.Entry(parent, textvariable=variable, width=entry_width)
        entry.grid(row=row, column=2, padx=5)
        
        # 입력값 검증 함수
        def validate_entry(*args):
            try:
                value = float(variable.get())
                if from_ <= value <= to_:
                    entry.configure(style="TEntry")  # 정상 스타일
                else:
                    entry.configure(style="Error.TEntry")  # 에러 스타일
            except ValueError:
                entry.configure(style="Error.TEntry")
        
        # 변수 변경 시 검증
        variable.trace('w', validate_entry)
        
        return scale, entry

    def create_widgets(self):
        # 에러 스타일 정의
        style = ttk.Style()
        style.configure("Error.TEntry", fieldbackground="lightcoral")
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 스크롤 가능한 프레임
        canvas = tk.Canvas(main_frame, height=700)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        
        # 파일 선택 섹션
        self.create_file_section(scrollable_frame, 0)
        
        # 디노이징 파라미터 섹션
        self.create_denoise_section(scrollable_frame, 1)
        
        # 스무딩 파라미터 섹션
        self.create_smoothing_section(scrollable_frame, 2)
        
        # 기본 처리 파라미터 섹션
        self.create_basic_section(scrollable_frame, 3)
        
        # 제어 버튼 섹션
        self.create_control_section(scrollable_frame, 4)
        
        # 그리드 웨이트 설정
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
    def create_file_section(self, parent, row):
        frame = ttk.LabelFrame(parent, text="파일 설정", padding="10")
        frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # PGM 파일
        ttk.Label(frame, text="PGM 트랙 이미지:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.pgm_var = tk.StringVar(value=self.params['pgm_path'])
        ttk.Entry(frame, textvariable=self.pgm_var, width=50).grid(row=0, column=1, padx=5)
        ttk.Button(frame, text="찾기", command=self.browse_pgm).grid(row=0, column=2)
        
        # YAML 파일
        ttk.Label(frame, text="YAML 정보 파일:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.yaml_var = tk.StringVar(value=self.params['yaml_path'])
        ttk.Entry(frame, textvariable=self.yaml_var, width=50).grid(row=1, column=1, padx=5)
        ttk.Button(frame, text="찾기", command=self.browse_yaml).grid(row=1, column=2)
        
        # 출력 디렉토리
        ttk.Label(frame, text="출력 디렉토리:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.output_var = tk.StringVar(value=self.params['output_dir'])
        ttk.Entry(frame, textvariable=self.output_var, width=50).grid(row=2, column=1, padx=5)
        ttk.Button(frame, text="찾기", command=self.browse_output).grid(row=2, column=2)
        
    def create_denoise_section(self, parent, row):
        frame = ttk.LabelFrame(parent, text="디노이징 파라미터", padding="10")
        frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # DBSCAN 반지름
        self.denoise_radius_var = tk.DoubleVar(value=self.params['denoise_radius'])
        self.create_slider_with_entry(frame, 0, "DBSCAN 반지름 (eps):", 
                                     self.denoise_radius_var, 0.5, 5.0)
        
        # 최소 샘플 비율
        self.denoise_min_samples_var = tk.DoubleVar(value=self.params['denoise_min_samples_ratio'])
        self.create_slider_with_entry(frame, 1, "최소 샘플 비율:", 
                                     self.denoise_min_samples_var, 2.0, 20.0)
        
        # 최소 클러스터 비율
        self.denoise_cluster_ratio_var = tk.DoubleVar(value=self.params['denoise_min_cluster_ratio'])
        self.create_slider_with_entry(frame, 2, "최소 클러스터 비율:", 
                                     self.denoise_cluster_ratio_var, 0.001, 0.1, resolution=0.001)
        
        # 최소 클러스터 크기
        self.denoise_cluster_min_var = tk.IntVar(value=self.params['denoise_min_cluster_min'])
        self.create_slider_with_entry(frame, 3, "최소 클러스터 크기:", 
                                     self.denoise_cluster_min_var, 5, 100, resolution=1)
        
    def create_smoothing_section(self, parent, row):
        frame = ttk.LabelFrame(parent, text="스무딩 파라미터", padding="10")
        frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 스무딩 방법
        ttk.Label(frame, text="스무딩 방법:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.smoothing_method_var = tk.StringVar(value=self.params['smoothing_method'])
        methods = ['none', 'curvature', 'gaussian', 'spline', 'both']
        ttk.Combobox(frame, textvariable=self.smoothing_method_var, values=methods, 
                    width=15).grid(row=0, column=1, sticky=tk.W, padx=5)
        
        # 곡률 임계값
        self.curvature_threshold_var = tk.DoubleVar(value=self.params['curvature_threshold'])
        self.create_slider_with_entry(frame, 1, "곡률 임계값:", 
                                     self.curvature_threshold_var, 0.1, 1.0)
        
        # 가우시안 시그마
        self.gaussian_sigma_var = tk.DoubleVar(value=self.params['gaussian_sigma'])
        self.create_slider_with_entry(frame, 2, "가우시안 시그마:", 
                                     self.gaussian_sigma_var, 0.5, 5.0)
        
        # 스플라인 포인트 비율 (더 넓은 범위로 고밀도 지원)
        self.spline_points_var = tk.DoubleVar(value=self.params['spline_points_ratio'])
        self.create_slider_with_entry(frame, 3, "스플라인 포인트 비율 (고밀도):", 
                                     self.spline_points_var, 0.5, 10.0)
        
        # 스무딩 윈도우 크기
        self.window_size_var = tk.IntVar(value=self.params['smoothing_window_size'])
        self.create_slider_with_entry(frame, 4, "스무딩 윈도우 크기:", 
                                     self.window_size_var, 10, 200, resolution=1)
        
        # 스무딩 반복 횟수
        self.iterations_var = tk.IntVar(value=self.params['smoothing_iterations'])
        self.create_slider_with_entry(frame, 5, "스무딩 반복 횟수:", 
                                     self.iterations_var, 1, 20, resolution=1)
        
        # 블렌딩 팩터
        self.blend_factor_var = tk.DoubleVar(value=self.params['smoothing_blend_factor'])
        self.create_slider_with_entry(frame, 6, "블렌딩 팩터:", 
                                     self.blend_factor_var, 0.1, 1.0)
        
    def create_basic_section(self, parent, row):
        frame = ttk.LabelFrame(parent, text="기본 처리 파라미터", padding="10")
        frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 서브샘플링 주기
        self.subsample_var = tk.IntVar(value=self.params['subsample_period'])
        self.create_slider_with_entry(frame, 0, "서브샘플링 주기:", 
                                     self.subsample_var, 1, 5)
        
        # 플롯 모드
        ttk.Label(frame, text="플롯 모드:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.plot_mode_var = tk.IntVar(value=self.params['plot_mode'])
        plot_modes = [("없음", 0), ("기본", 1), ("전체", 2)]
        for i, (text, value) in enumerate(plot_modes):
            ttk.Radiobutton(frame, text=text, variable=self.plot_mode_var, 
                           value=value).grid(row=1, column=i+1, padx=5)
        
        # 트랙 폭 계산 옵션
        self.calculate_widths_var = tk.BooleanVar(value=self.params['calculate_track_widths'])
        ttk.Checkbutton(frame, text="트랙 폭 계산 (좌우 거리 포함)", 
                       variable=self.calculate_widths_var).grid(row=2, column=0, columnspan=4, sticky=tk.W, pady=5)
    
    def create_control_section(self, parent, row):
        frame = ttk.Frame(parent, padding="10")
        frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=10)
        
        # 버튼들
        ttk.Button(frame, text="설정 저장", command=self.save_config).grid(row=0, column=0, padx=5)
        ttk.Button(frame, text="설정 로드", command=self.load_config).grid(row=0, column=1, padx=5)
        ttk.Button(frame, text="기본값 복원", command=self.reset_to_defaults).grid(row=0, column=2, padx=5)
        ttk.Button(frame, text="실행", command=self.run_processing, 
                  style="Accent.TButton").grid(row=0, column=3, padx=5)
        ttk.Button(frame, text="결과 시각화", command=self.show_visualization).grid(row=0, column=4, padx=5)
        
        # 프로그레스 바
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(frame, variable=self.progress_var, 
                                          maximum=100, length=300)
        self.progress_bar.grid(row=1, column=0, columnspan=4, pady=10)
        
        # 상태 라벨
        self.status_var = tk.StringVar(value="준비됨")
        ttk.Label(frame, textvariable=self.status_var).grid(row=2, column=0, columnspan=4)
    
    def browse_pgm(self):
        filename = filedialog.askopenfilename(
            title="PGM 트랙 이미지 파일 선택",
            filetypes=[("PGM files", "*.pgm"), ("All files", "*.*")]
        )
        if filename:
            self.pgm_var.set(filename)
    
    def browse_yaml(self):
        filename = filedialog.askopenfilename(
            title="YAML 정보 파일 선택",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if filename:
            self.yaml_var.set(filename)
    
    def browse_output(self):
        dirname = filedialog.askdirectory(title="출력 디렉토리 선택")
        if dirname:
            self.output_var.set(dirname)
    
    def get_current_params(self):
        """현재 GUI의 파라미터 값들을 반환"""
        return {
            'pgm_path': self.pgm_var.get(),
            'yaml_path': self.yaml_var.get(),
            'output_dir': self.output_var.get(),
            'denoise_radius': self.denoise_radius_var.get(),
            'denoise_min_samples_ratio': self.denoise_min_samples_var.get(),
            'denoise_min_cluster_ratio': self.denoise_cluster_ratio_var.get(),
            'denoise_min_cluster_min': self.denoise_cluster_min_var.get(),
            'smoothing_method': self.smoothing_method_var.get(),
            'curvature_threshold': self.curvature_threshold_var.get(),
            'gaussian_sigma': self.gaussian_sigma_var.get(),
            'spline_points_ratio': self.spline_points_var.get(),
            'smoothing_window_size': self.window_size_var.get(),
            'smoothing_iterations': self.iterations_var.get(),
            'smoothing_blend_factor': self.blend_factor_var.get(),
            'subsample_period': self.subsample_var.get(),
            'plot_mode': self.plot_mode_var.get(),
            'calculate_track_widths': self.calculate_widths_var.get()
        }
    
    def save_config(self):
        """현재 설정을 파일로 저장"""
        filename = filedialog.asksaveasfilename(
            title="설정 파일 저장",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(self.get_current_params(), f, indent=2, ensure_ascii=False)
                messagebox.showinfo("성공", "설정이 저장되었습니다.")
            except Exception as e:
                messagebox.showerror("오류", f"설정 저장 실패: {e}")
    
    def load_config(self):
        """설정 파일 로드"""
        filename = filedialog.askopenfilename(
            title="설정 파일 로드",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    params = json.load(f)
                self.apply_params(params)
                messagebox.showinfo("성공", "설정이 로드되었습니다.")
            except Exception as e:
                messagebox.showerror("오류", f"설정 로드 실패: {e}")
    
    def apply_params(self, params):
        """파라미터를 GUI에 적용"""
        self.pgm_var.set(params.get('pgm_path', ''))
        self.yaml_var.set(params.get('yaml_path', ''))
        self.output_var.set(params.get('output_dir', '/home/f1/f1tenth_ws/joon_path_generate/final_centerline/output/'))
        self.denoise_radius_var.set(params.get('denoise_radius', 2.0))
        self.denoise_min_samples_var.set(params.get('denoise_min_samples_ratio', 8.0))
        self.denoise_cluster_ratio_var.set(params.get('denoise_min_cluster_ratio', 0.01))
        self.denoise_cluster_min_var.set(params.get('denoise_min_cluster_min', 10))
        self.smoothing_method_var.set(params.get('smoothing_method', 'curvature'))
        self.curvature_threshold_var.set(params.get('curvature_threshold', 0.3))
        self.gaussian_sigma_var.set(params.get('gaussian_sigma', 2.0))
        self.spline_points_var.set(params.get('spline_points_ratio', 2.0))
        self.window_size_var.set(params.get('smoothing_window_size', 60))
        self.iterations_var.set(params.get('smoothing_iterations', 5))
        self.blend_factor_var.set(params.get('smoothing_blend_factor', 0.3))
        self.subsample_var.set(params.get('subsample_period', 1))
        self.plot_mode_var.set(params.get('plot_mode', 1))
        self.calculate_widths_var.set(params.get('calculate_track_widths', False))
    
    def load_default_params(self):
        """기본 파라미터 로드"""
        self.apply_params(self.params)
    
    def reset_to_defaults(self):
        """기본값으로 초기화"""
        self.apply_params(self.params)
        messagebox.showinfo("완료", "기본값으로 복원되었습니다.")
    
    def run_processing(self):
        """centerline fitting 실행"""
        current_params = self.get_current_params()
        
        # 필수 파일 확인
        if not current_params['pgm_path'] or not current_params['yaml_path']:
            messagebox.showerror("오류", "PGM 파일과 YAML 파일을 모두 선택해주세요.")
            return
        
        if not os.path.exists(current_params['pgm_path']):
            messagebox.showerror("오류", "PGM 파일이 존재하지 않습니다.")
            return
            
        if not os.path.exists(current_params['yaml_path']):
            messagebox.showerror("오류", "YAML 파일이 존재하지 않습니다.")
            return
        
        # 처리 실행
        self.status_var.set("처리 중...")
        self.progress_var.set(0)
        self.root.update()
        
        try:
            # fit_centerline_gui.py 모듈 import 및 실행
            from fit_centerline_gui import process_centerline_with_params
            
            def progress_callback(percent, message=""):
                self.progress_var.set(percent)
                self.status_var.set(message)
                self.root.update()
            
            result = process_centerline_with_params(current_params, progress_callback)
            
            if result:
                self.progress_var.set(100)
                self.status_var.set("완료")
                
                # 결과 파일 저장
                self.last_result_file = result
                self.last_pgm_file = current_params['pgm_path']
                
                messagebox.showinfo("성공", f"처리가 완료되었습니다.\n출력 파일: {result}\n\n'결과 시각화' 버튼을 클릭하여 PGM 위에 centerline을 확인할 수 있습니다.")
            else:
                self.status_var.set("실패")
                messagebox.showerror("오류", "처리 중 오류가 발생했습니다.")
                
        except Exception as e:
            self.status_var.set("실패")
            messagebox.showerror("오류", f"처리 실패: {e}")
            self.progress_var.set(0)
    
    def show_visualization(self):
        """PGM 위에 centerline을 오버레이하여 시각화"""
        if not self.last_result_file or not self.last_pgm_file:
            # 파일 선택 다이얼로그 표시
            if not self.last_pgm_file:
                self.last_pgm_file = self.pgm_var.get()
            
            if not self.last_pgm_file or not os.path.exists(self.last_pgm_file):
                messagebox.showwarning("Warning", "Please run processing first or select a PGM file.")
                return
                
            # CSV 파일 선택
            csv_file = filedialog.askopenfilename(
                title="Select Centerline CSV File",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                initialdir=self.output_var.get()
            )
            if not csv_file:
                return
            self.last_result_file = csv_file
        
        try:
            self.create_visualization_window()
        except Exception as e:
            messagebox.showerror("Error", f"Visualization failed: {e}")
    
    def create_visualization_window(self):
        """시각화 창 생성"""
        viz_window = Toplevel(self.root)
        viz_window.title("Centerline 시각화")
        viz_window.geometry("1000x800")
        
        # matplotlib figure 생성
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        try:
            # PGM 파일 로드
            image = self.load_pgm_image(self.last_pgm_file)
            
            # YAML 파일에서 변환 정보 로드 
            yaml_path = self.last_pgm_file.replace('.pgm', '.yaml')
            if os.path.exists(yaml_path):
                resolution, origin = self.load_yaml_info(yaml_path)
            else:
                resolution, origin = 0.05, [0, 0, 0]  # 기본값
            
            # CSV 파일 로드 (헤더 자동 감지 및 예외 처리 강화)
            centerline_data = None
            
            # 1. 파일 존재 여부 확인
            if not os.path.exists(self.last_result_file):
                messagebox.showerror("파일 오류", f"결과 CSV 파일을 찾을 수 없습니다:\n{self.last_result_file}")
                viz_window.destroy() # 오류 발생 시 시각화 창 닫기
                return

            try:
                # 2. 헤더가 없다고 가정하고 우선 로드 시도
                centerline_data = np.loadtxt(self.last_result_file, delimiter=',')
                print("✅ CSV 파일 로드 성공 (헤더 없음).")
            except ValueError:
                # 3. ValueError 발생 시 -> 헤더가 있을 가능성이 높음
                print("⚠️ CSV 헤더 감지됨, 첫 번째 행을 스킵하고 다시 로드합니다.")
                try:
                    # 헤더를 건너뛰고 다시 로드
                    centerline_data = np.loadtxt(self.last_result_file, delimiter=',', skiprows=1)
                    print("✅ CSV 파일 로드 성공 (헤더 스킵).")
                except Exception as inner_e:
                    # 헤더를 스킵해도 오류 발생 시
                    messagebox.showerror("CSV 로드 오류", f"헤더 스킵 후에도 파일 로드에 실패했습니다: {inner_e}")
                    viz_window.destroy()
                    return
            except Exception as e:
                # 4. 그 외의 파일 로드 오류 (e.g., 접근 권한)
                messagebox.showerror("파일 로드 오류", f"파일 로드 중 예상치 못한 오류가 발생했습니다: {e}")
                viz_window.destroy()
                return

            # 데이터 로드에 최종 실패한 경우 함수 종료
            if centerline_data is None:
                messagebox.showerror("오류", "원인 미상의 이유로 CSV 데이터 로드에 실패했습니다.")
                viz_window.destroy()
                return
            
            # CSV 형식 확인
            if centerline_data.ndim == 1:
                centerline_data = centerline_data.reshape(1, -1)
            
            has_track_widths = centerline_data.shape[1] >= 4
            
            if has_track_widths:
                # 확장된 형식 (x, y, left_dist, right_dist)
                centerline_points = centerline_data[:, :2]
                left_distances = centerline_data[:, 2]
                right_distances = centerline_data[:, 3]
                print(f"트랙 폭 정보 포함된 CSV 파일 감지: {len(centerline_points)}개 포인트")
            else:
                # 기본 형식 (x, y)
                centerline_points = centerline_data[:, :2]
                left_distances = None
                right_distances = None
                print(f"기본 CSV 파일 감지: {len(centerline_points)}개 포인트")
            
            # 픽셀 좌표로 변환
            pixel_points = self.world_to_pixel(centerline_points, resolution, origin, image.shape)
            
            # 첫 번째 서브플롯: PGM 이미지만
            ax1.imshow(image, cmap='gray', origin='upper')
            ax1.set_title('Original PGM Image')
            ax1.axis('off')
            
            # 두 번째 서브플롯: PGM + Centerline 오버레이
            ax2.imshow(image, cmap='gray', origin='upper', alpha=0.7)
            
            # Centerline 그리기
            if len(pixel_points) > 0:
                # 트랙 폭 정보가 있으면 트랙 경계도 그리기
                if has_track_widths:
                    self.draw_track_boundaries(ax2, centerline_points, left_distances, right_distances, resolution, origin, image.shape)
                
                # 닫힌 루프를 위해 첫 번째 점을 마지막에 추가
                closed_points = np.vstack([pixel_points, pixel_points[0:1]])
                ax2.plot(closed_points[:, 0], closed_points[:, 1], 'r-', linewidth=3, label='Centerline', alpha=0.8)
                ax2.plot(closed_points[:, 0], closed_points[:, 1], 'y-', linewidth=1, alpha=0.9)
                
                # 시작점 표시
                ax2.scatter(pixel_points[0, 0], pixel_points[0, 1], c='green', s=100, 
                           marker='s', label='Start Point', zorder=5, edgecolor='black', linewidth=2)
                
                # 포인트 표시 (더 많은 포인트 표시 - 전체의 20% 정도)
                show_ratio = 0.20  # 전체 포인트의 20%
                show_count = max(50, int(len(pixel_points) * show_ratio))
                step = max(1, len(pixel_points) // show_count)
                
                # 포인트들을 작은 원으로 표시
                show_indices = range(0, len(pixel_points), step)
                show_points = pixel_points[show_indices]
                ax2.scatter(show_points[:, 0], show_points[:, 1], 
                           c='cyan', s=8, alpha=0.7, zorder=6, 
                           edgecolor='blue', linewidth=0.3)
                
                # 방향 표시 (화살표는 더 적게)
                arrow_step = max(1, len(pixel_points) // 30)
                for i in range(0, len(pixel_points), arrow_step):
                    if i + 1 < len(pixel_points):
                        dx = pixel_points[i+1, 0] - pixel_points[i, 0]
                        dy = pixel_points[i+1, 1] - pixel_points[i, 1]
                        ax2.arrow(pixel_points[i, 0], pixel_points[i, 1], dx*0.3, dy*0.3,
                                 head_width=3, head_length=3, fc='blue', ec='blue', alpha=0.6)
            
            if has_track_widths:
                ax2.set_title(f'PGM + Centerline + Track Boundaries\n({len(centerline_points)} points with track widths)')
            else:
                ax2.set_title(f'PGM + Centerline Overlay\n({len(centerline_points)} points)')
            ax2.legend(loc='upper right')
            ax2.axis('off')
            
            plt.tight_layout()
            
            # Tkinter 캔버스에 matplotlib 그림 임베드
            canvas = FigureCanvasTkAgg(fig, viz_window)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # 정보 표시 프레임
            info_frame = ttk.Frame(viz_window)
            info_frame.pack(fill=tk.X, padx=10, pady=5)
            
            info_text = f"PGM: {os.path.basename(self.last_pgm_file)} | "
            info_text += f"CSV: {os.path.basename(self.last_result_file)} | "
            info_text += f"Points: {len(centerline_points)} | "
            info_text += f"Resolution: {resolution} m/pixel"
            
            ttk.Label(info_frame, text=info_text, font=('Arial', 9)).pack()
            
            # 버튼 프레임
            button_frame = ttk.Frame(viz_window)
            button_frame.pack(fill=tk.X, padx=10, pady=5)
            
            ttk.Button(button_frame, text="Save Image", 
                      command=lambda: self.save_visualization(fig)).pack(side=tk.LEFT, padx=5)
            ttk.Button(button_frame, text="Close", 
                      command=viz_window.destroy).pack(side=tk.RIGHT, padx=5)
            
        except Exception as e:
            messagebox.showerror("Visualization Error", f"Failed to create visualization: {e}")
            viz_window.destroy()
    
    def load_pgm_image(self, pgm_path):
        """PGM 이미지 로드"""
        with open(pgm_path, 'rb') as f:
            # PGM 헤더 파싱
            header = []
            while len(header) < 4:
                line = f.readline()
                words = line.split()
                if len(words) > 0 and words[0] != b'#':
                    header.extend(words)
            
            width = int(header[1])
            height = int(header[2])
            
            # 이미지 데이터 읽기
            image_data = []
            for _ in range(height):
                row = []
                for _ in range(width):
                    byte = f.read(1)
                    if byte:
                        row.append(ord(byte))
                    else:
                        row.append(0)
                image_data.append(row)
            
            return np.array(image_data)
    
    def load_yaml_info(self, yaml_path):
        """YAML 파일에서 변환 정보 로드"""
        import yaml
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            resolution = data.get('resolution', 0.05)
            origin = data.get('origin', [0, 0, 0])
            return resolution, origin
    
    def world_to_pixel(self, world_points, resolution, origin, image_shape):
        """월드 좌표를 픽셀 좌표로 변환"""
        # 원래 centerline 생성 과정:
        # 1. 픽셀 좌표 -> 월드 좌표: track_cycle = track_cycle * resolution - origin[0:2]
        # 2. 역변환 (월드 좌표 -> 픽셀 좌표): pixel_coords = (world_coords + origin[0:2]) / resolution
        
        # 월드 좌표를 픽셀 좌표로 변환
        pixel_points = (world_points + np.array(origin[:2])) / resolution
        
        # Y축 뒤집기 제거 - 직접 매핑 테스트
        # pixel_points[:, 1] = image_shape[0] - pixel_points[:, 1]
        
        return pixel_points
    
    def draw_track_boundaries(self, ax, centerline_points, left_distances, right_distances, resolution, origin, image_shape):
        """트랙 경계선을 시각화 (Raceline-Optimization 호환 방식)"""
        print("트랙 경계선 시각화 중... (Raceline-Optimization 호환 법선벡터 사용)")
        
        left_boundary_points = []
        right_boundary_points = []
        
        for i, point in enumerate(centerline_points):
            # Raceline-Optimization 방식: 인접점 기반 방향 벡터 계산 (트랙 폭 계산과 동일한 방법)
            current_point = point
            
            if i == 0:
                # 첫 번째 점: 다음 점 방향
                if len(centerline_points) > 1:
                    direction_vector = np.array([
                        centerline_points[1][0] - centerline_points[0][0], 
                        centerline_points[1][1] - centerline_points[0][1]
                    ])
                else:
                    direction_vector = np.array([1, 0])
            elif i == len(centerline_points) - 1:
                # 마지막 점: 이전 점에서의 방향
                direction_vector = np.array([
                    centerline_points[i][0] - centerline_points[i-1][0], 
                    centerline_points[i][1] - centerline_points[i-1][1]
                ])
            else:
                # 중간 점: 이전과 다음 점 사이의 방향 (Raceline-Optimization 방식)
                direction_vector = np.array([
                    centerline_points[i+1][0] - centerline_points[i-1][0], 
                    centerline_points[i+1][1] - centerline_points[i-1][1]
                ])
            
            # 벡터 정규화
            if np.linalg.norm(direction_vector) > 0:
                direction_vector = direction_vector / np.linalg.norm(direction_vector)
            else:
                direction_vector = np.array([1, 0])
            
            # 법선 벡터 계산 (좌우 방향)
            left_normal = np.array([-direction_vector[1], direction_vector[0]])
            right_normal = np.array([direction_vector[1], -direction_vector[0]])
            
            # 좌우 경계 포인트 계산 (실제 좌표계에서)
            left_point = current_point + left_normal * left_distances[i]
            right_point = current_point + right_normal * right_distances[i]
            
            left_boundary_points.append(left_point)
            right_boundary_points.append(right_point)
        
        # 실제 좌표를 픽셀 좌표로 변환
        left_boundary_pixels = self.world_to_pixel(np.array(left_boundary_points), resolution, origin, image_shape)
        right_boundary_pixels = self.world_to_pixel(np.array(right_boundary_points), resolution, origin, image_shape)
        
        # 닫힌 루프로 만들기
        left_boundary_closed = np.vstack([left_boundary_pixels, left_boundary_pixels[0:1]])
        right_boundary_closed = np.vstack([right_boundary_pixels, right_boundary_pixels[0:1]])
        
        # 경계선 그리기
        ax.plot(left_boundary_closed[:, 0], left_boundary_closed[:, 1], 'g--', 
               linewidth=2, label='Left Boundary', alpha=0.8)
        ax.plot(right_boundary_closed[:, 0], right_boundary_closed[:, 1], 'b--', 
               linewidth=2, label='Right Boundary', alpha=0.8)
        
        # 몇 개 포인트에서 centerline과 경계를 연결하는 선 그리기
        step = max(1, len(centerline_points) // 20)  # 20개 정도만 표시
        centerline_pixels = self.world_to_pixel(centerline_points, resolution, origin, image_shape)
        
        for i in range(0, len(centerline_points), step):
            # centerline에서 좌측 경계로
            ax.plot([centerline_pixels[i, 0], left_boundary_pixels[i, 0]], 
                   [centerline_pixels[i, 1], left_boundary_pixels[i, 1]], 
                   'g-', linewidth=0.5, alpha=0.4)
            # centerline에서 우측 경계로
            ax.plot([centerline_pixels[i, 0], right_boundary_pixels[i, 0]], 
                   [centerline_pixels[i, 1], right_boundary_pixels[i, 1]], 
                   'b-', linewidth=0.5, alpha=0.4)
        
        print(f"트랙 경계선 시각화 완료: 좌측 {len(left_boundary_points)}개, 우측 {len(right_boundary_points)}개 포인트")
    
    def save_visualization(self, fig):
        """Save visualization result as image file"""
        file_path = filedialog.asksaveasfilename(
            title="Save Visualization Image",
            defaultextension=".png",
            filetypes=[("PNG files", "*.png"), ("JPG files", "*.jpg"), ("All files", "*.*")]
        )
        if file_path:
            fig.savefig(file_path, dpi=300, bbox_inches='tight')
            messagebox.showinfo("Save Complete", f"Visualization image saved:\n{file_path}")

def main():
    root = tk.Tk()
    app = ParameterGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
