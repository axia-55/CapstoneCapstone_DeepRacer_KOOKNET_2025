# KOOKNET DeepRacer Capstone 2025 🏎️

> 2025년도 1학기 **국민대학교 캡스톤디자인** KOOKNET 팀  
> AWS DeepRacer를 활용한 강화학습 기반 자율주행 경진대회 프로젝트

[📄 대회 설명 PDF 전체 보기](docs/competition_overview.pdf)

![대회 포스터](https://github.com/user-attachments/assets/51c46838-48c7-4dd6-9db1-c1483941b192)

---

## 대회 개요
- **일정** | 2025-05-27(화) 19:00  
- **장소** | 국민대학교 산학협력관 지하 1층  
- **부문** | 🔹 강화학습 알고리즘 부문 🔹 자유 알고리즘 부문 :contentReference[oaicite:4]{index=4}

---

## 라운드별 규칙

<details>
<summary>🔹 Round 1 – Reinforcement Learning Time Trial</summary>

| 항목 | 내용 |
| ---- | ---- |
| Track | RL Speedway (95 % 축소) |
| 방향 | 반시계 |
| 알고리즘 | 강화학습 |
| 레이스 | Time Trial |
| 패널티 | 트랙 아웃 +2 s |
| 미션 | 3 분 세션×2, 세션당 Timeout 1 분, **가장 빠른 한 바퀴** 기록 기준 |

<img src="https://github.com/user-attachments/assets/ef559142-09d7-491b-8aae-72cecf27228f" width="600">
</details>

<details>
<summary>🔹 Round 2 – Free Algorithm Object Avoidance</summary>

| 항목 | 내용 |
| ---- | ---- |
| Track | RL Speedway (95 %) |
| 방향 | 반시계 |
| 알고리즘 | 자유 |
| 레이스 | Object Avoidance |
| 패널티 | 트랙 아웃 +2 s |
| 미션 | 조건 동일 (3 분 세션×2, Timeout 1 분) |

<img src="https://github.com/user-attachments/assets/1025cd03-8777-43d7-998d-2409c0b653ad" width="600">
</details>

<details>
<summary>🔹 Round 3 – Secret Track Time Trial</summary>

| 항목 | 내용 |
| ---- | ---- |
| Track | **대회 당일 공개** |
| 방향 | 반시계 |
| 알고리즘 | 강화학습 |
| 레이스 | Time Trial |
| 패널티 | 트랙 아웃 +2 s |
| 미션 | 조건 동일 |

<img src="https://github.com/user-attachments/assets/561b41ec-efad-46e8-8efb-d018c2322540" width="600">
</details>

---

## 공통 규정 (요약)

- **계측** | 타이밍 프로그램(조교), 주행 영상 녹화  
- **주행** | 스타트·피니시 모두 전방 바퀴 **시작선** 기준, 트랙 이탈 시 마지막 지점에 재위치  
- **Timeout** | 세션당 최대 1 분 (재부팅 포함)

---

## 저장소 구성
-본 프로젝트 저장소는 **라운드(Round) → 모델(Model)** 2-단계 디렉터리 구조로 정리되어 있습니다. 
-현재 reward_function만 올라가 있습니다.
