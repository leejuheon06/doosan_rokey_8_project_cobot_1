# 로봇 상태 (로봇 -> UI)
# 각 단계가 바뀔 때마다 로봇 노드가 발행. UI는 이 값 보고 화면 갱신.
#
# 상태 흐름:
#   WAITING(0) --메뉴 수신--> MAKING(1) --재료 다 담고 마지막 모션까지--> MAKING_DONE(2)
#   --> DELIVERING(3) --지정 좌표에 놓고 ungrip--> DELIVERED(4)
#   --> RETURNING_HOME(5) --홈 도착--> WAITING(0)

int32 WAITING         = 0    # 대기: UI에서 손님 입력 없으면 계속 대기
int32 MAKING          = 1    # 제조 중: 재료 담기 + 마지막 모션까지
int32 MAKING_DONE     = 2    # 제조 완료: 모션까지 끝남
int32 DELIVERING      = 3    # 전달 중: 완료 후 테이블로 컵 들고 이동
int32 DELIVERED       = 4    # 전달 완료: 지정 좌표에 놓고 ungrip 완료
int32 RETURNING_HOME  = 5    # 홈 복귀: 홈 좌표 복귀 중 (도착하면 WAITING 으로)

int32 status                 # 위 여섯 값 중 하나
