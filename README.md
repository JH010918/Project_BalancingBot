# 밸런싱봇
사용자가 웹 페이지에서 촬용한 방 사진을 AI가 분석하여 청결 상태를 판단하고 점수로 제공하는 서비스
## 📌 프로젝트 개요
- 수행 기간 : 2026.03.06 ~ 2026.03.20
- 팀원 : 김병현, 김권, 김동석
- 배경 : 1인 가구 증가에 따라 많은 사람들이 청소를 미룬다. 청소 타이밍을 놓치게 되면 방은 엉망이 되기 때문에 방 상태의 기준을 세워 방 청소에 도움이 되고자 한다.
- 목표 :
  - 정리 구역 시각화
  - 객관적 청결 지수 수립
- 기대 효과 :
  - 청결 관리 동기 부여
  - 방 정리 습관 형성
  - 삶의 질 향상
- 주요 기능 :
  - 이미지 업로드 및 청결도 점수화 (0~100점)
  - Grad-CAM을 이용한 dirty 구역 시각화 및 YOLO 기반 객체 분포 분석
  - CLIP 기반의 방 상태 요약
- 이미지 데이터 : https://drive.google.com/drive/folders/1xH8Oj7zAddebDz-nC4Pb0cQJk2OXOPtT?usp=drive_link
## 🔧 기술 스택
| 기술 | 설명 |
|----------|------------|
| Python | 전체 시스템 및 AI 모델 개발 |
| Pytorch, TensorFlow,OpenAI CLIP| 이미지를 학습하여 청결도를 분류하는 딥러닝 모델 학습 |
| OpenCV | 이미지 로드, 크기 조정, 전처리 수행 |
| Flask | AI 모델을 제공하는 웹 서버 실행 |
| MYSQL | 커뮤니 데이터베이스 관리 |

## ⚙️ 시스템 아키텍처
![system_architecture](img/system_architecture.png)

## 💾 데이터
### 1️⃣ 수집
- Python을 사용한 방/책상 사진 웹 크롤링
- 사람, 워터마크 등 포함된 이미지 제거
### 2️⃣ 전처리
  - 이미지 표준화 : 해상도 256px * 256px, jpg 포맷
  - 데이터 증강 (회전, 반)
  - gray scale 적용
### 3️⃣ 라벨링
- clean
- dirty
  - 판단 기준 : 10.1017/S1041610209990135 참고

## 🎥 시연
![uiux1](img/ui_ux1.jpg)
![uiux1](img/ui_ux2.png)
![uiux1](img/ui_ux3.png)
![uiux1](img/ui_ux4.png)

## 🚨 트러블슈팅
### 1️⃣ 이미지 오분류 문제
![ts](img/ts1.jpg)
### 2️⃣ 과적합 문제
![ts](img/ts2.jpg)
### 3️⃣ 데이터셋 편향 문제
![ts](img/ts3.jpg)
### 4️⃣ tensorflow 버전 차이 문제
![ts](img/ts4.jpg)
