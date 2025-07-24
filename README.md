# Visual Odometry Implementation

이 프로젝트는 단안(Monocular) 및 스테레오(Stereo) 비전을 이용한 Visual Odometry 구현을 포함합니다.

## 프로젝트 구조

```
VisualOdometry/
├── monoVO/          # 단안 카메라 기반 Visual Odometry
│   ├── main.py                    # 메인 실행 파일
│   ├── visual_odometry_ORB.py     # ORB 특징점 기반 VO 구현
│   ├── visual_odometry_FAST.py    # FAST 특징점 기반 VO 구현
│   └── map.png                    # 생성된 궤적 이미지
└── stereoVO/        # 스테레오 카메라 기반 Visual Odometry
    ├── main.py                    # 메인 실행 파일
    ├── visual_odometry_FAST.py    # FAST 특징점 기반 스테레오 VO 구현
    ├── helperFunctions.py         # 도움 함수들
    ├── inlierDetector.py          # 인라이어 검출 모듈
    └── map.png                    # 생성된 궤적 이미지
```

## 기능

### MonoVO (단안 Visual Odometry)
- **ORB 특징점 기반**: `visual_odometry_ORB.py`를 사용한 특징점 매칭
- **FAST 특징점 기반**: `visual_odometry_FAST.py`를 사용한 광학 흐름 추적
- KITTI 데이터셋 지원
- 실시간 궤적 시각화

### StereoVO (스테레오 Visual Odometry)
- **스테레오 비전**: 좌우 카메라 이미지를 이용한 깊이 정보 활용
- **FAST 특징점**: KLT 광학 흐름을 이용한 특징점 추적
- **인라이어 검출**: RANSAC 기반 아웃라이어 제거
- **3D 점 생성**: 스테레오 매칭을 통한 3차원 점 생성
- **포즈 최적화**: 재투영 오차 최소화를 통한 카메라 포즈 추정

## 요구사항

```bash
pip install numpy opencv-python scipy
```

## 사용법

### 단안 VO 실행
```bash
cd monoVO
python main.py
```

### 스테레오 VO 실행
```bash
cd stereoVO
python main.py
```

## 데이터셋

이 구현은 KITTI Visual Odometry 데이터셋을 기준으로 작성되었습니다.
- KITTI 데이터셋 경로를 코드 내에서 수정하여 사용하세요.
- 카메라 캘리브레이션 매개변수는 KITTI 데이터셋 기준입니다.

## 카메라 매개변수

```python
# KITTI 데이터셋 기본 매개변수
width = 1241.0
height = 376.0  
fx = 718.8560
fy = 718.8560
cx = 607.1928
cy = 185.2157
```

## 결과

실행 후 다음과 같은 결과를 확인할 수 있습니다:
- 실시간 카메라 이미지 표시
- 추정된 궤적의 실시간 시각화
- 최종 궤적이 `map.png`로 저장

## 특징

- **실시간 처리**: OpenCV를 이용한 효율적인 이미지 처리
- **시각화**: 궤적의 실시간 시각화 및 좌표 표시
- **모듈화**: 재사용 가능한 클래스 구조
- **확장성**: 다양한 특징점 검출기 지원

## 기술적 세부사항

### 단안 VO
- **특징점 추출**: ORB 또는 FAST 검출기
- **매칭**: BFMatcher (ORB) 또는 KLT 추적 (FAST)
- **포즈 추정**: Essential Matrix 분해

### 스테레오 VO
- **스테레오 매칭**: 좌우 이미지 간 특징점 매칭
- **삼각측량**: 3D 점 복원
- **포즈 추정**: PnP 알고리즘 및 번들 조정
- **최적화**: Levenberg-Marquardt 알고리즘

## 라이선스

MIT License

## 기여

이슈나 개선 사항이 있으시면 언제든지 Pull Request를 보내주세요.
