#!/usr/bin/env python3
# =============================================================================
#  rate_sign_check.py — pitch 각속도 부호 벤치 확인 (2026-10-31)
#
#  왜 필요한가:
#    pid_control_node 의 종속(각속도) 제어는
#        u_pitch = kp_pitch * (각도오차 + kp_pitch_rate * 각속도오차)
#    인데, 각속도 부호가 반대면 **제동이 가속이 된다** (출렁임이 두 배로 커진다).
#    이 로봇은 **양수 pitch = 기수 아래**로 항공 관례와 반대고(2026-08-04 실측),
#    센서 장착·축 변환·부호 규약 세 군데서 뒤집힐 수 있어 코드만 보고는 못 가린다.
#    CLAUDE.md Known gaps: "pitch PID 건드리기 전에 부호 확인".
#
#  무엇을 재는가:
#    /filtered/attitude 의 pitch 와 /filtered/angular_rate 의 y 를 같이 받아,
#    **pitch 가 실제로 증가한 구간에서 각속도 y 가 양수인지**를 본다.
#    사람이 "기수를 내렸다" 고 말할 필요가 없다 — 자세 변화 자체를 기준으로 삼으므로
#    어느 방향으로 기울이든 판정이 같다. 이게 눈으로 숫자를 읽는 것보다 안전하다.
#
#  판정: 두 신호의 상관계수 r
#    r > +0.8  정상 (같은 방향)        -> kp_pitch_rate 써도 됨
#    r < -0.8  반대 (부호 뒤집힘)      -> 쓰면 발산. 코드 수정 필요
#    그 사이    흔들기가 부족           -> 더 크게/빠르게 다시
# =============================================================================
import sys, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

SETTLE_SEC   = 1.0    # 기동 직후 버리는 시간
MIN_SAMPLES  = 200    # 최소 2초
MIN_PITCH_PP = 8.0    # pitch 변화 폭이 이보다 작으면 "덜 흔들었다"
MIN_RATE_PP  = 10.0   # 각속도 변화 폭 [도/초]

class SignCheck(Node):
    def __init__(self, dur):
        super().__init__('rate_sign_check')
        self.dur = dur
        self.pitch = None
        self.rows = []          # (pitch, rate_y)
        self.t0 = time.time()
        self.create_subscription(Vector3, '/filtered/attitude', self.on_att, 10)
        self.create_subscription(Vector3, '/filtered/angular_rate', self.on_rate, 50)
        self.create_timer(0.5, self.tick)
        self.last_print = 0

    def on_att(self, m):
        # 모드 인코딩(AUTO 는 roll 에 ±5000) 은 pitch 에 영향 없다
        self.pitch = m.y

    def on_rate(self, m):
        if self.pitch is None:
            return
        if time.time() - self.t0 < SETTLE_SEC:
            return
        self.rows.append((self.pitch, m.y))

    def tick(self):
        el = time.time() - self.t0
        if self.pitch is None and el > 3.0:
            print('  대기 중... /filtered/attitude 가 안 온다 (EKF 기동 중?)')
            return
        if len(self.rows) > self.last_print + 100:
            self.last_print = len(self.rows)
            p = [r[0] for r in self.rows]
            print(f'  수집 {len(self.rows):5d}  pitch 범위 {min(p):+6.1f}~{max(p):+6.1f}도')

def corr(xs, ys):
    n = len(xs)
    mx, my = sum(xs)/n, sum(ys)/n
    sxy = sum((a-mx)*(b-my) for a, b in zip(xs, ys))
    sxx = sum((a-mx)**2 for a in xs)
    syy = sum((b-my)**2 for b in ys)
    if sxx <= 0 or syy <= 0:
        return None
    return sxy / math.sqrt(sxx*syy)

def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    print('=' * 68)
    print(' pitch 각속도 부호 확인')
    print('=' * 68)
    print(f' {dur:.0f}초 동안 로봇의 **기수를 손으로 위아래로 천천히 흔들어** 주세요.')
    print(' (한 번에 15도 이상, 2~3회 왕복. 좌우(roll)는 흔들지 마세요)')
    print('=' * 68)

    rclpy.init()
    node = SignCheck(dur)
    try:
        while rclpy.ok() and time.time() - node.t0 < dur + SETTLE_SEC:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    print()
    if len(rows) < MIN_SAMPLES:
        print(f'[실패] 표본 {len(rows)}개 (필요 {MIN_SAMPLES}). 토픽이 안 오거나 시간이 짧다.')
        return 2

    ps = [r[0] for r in rows]
    ws = [r[1] for r in rows]
    p_pp = max(ps) - min(ps)
    w_pp = max(ws) - min(ws)

    # pitch 의 수치 미분과 각속도를 비교한다 (둘 다 "변화" 를 보는 양)
    dps, wmid = [], []
    for i in range(1, len(rows)):
        dps.append(rows[i][0] - rows[i-1][0])
        wmid.append((rows[i][1] + rows[i-1][1]) / 2.0)

    r = corr(dps, wmid)

    print(f'표본            {len(rows)}개')
    print(f'pitch 변화 폭   {p_pp:.1f}도   (min {min(ps):+.1f}  max {max(ps):+.1f})')
    print(f'각속도 변화 폭  {w_pp:.1f}도/초 (min {min(ws):+.1f}  max {max(ws):+.1f})')
    print(f'상관계수 r      {r:+.3f}' if r is not None else '상관계수 r      계산불가')
    print()

    if p_pp < MIN_PITCH_PP or w_pp < MIN_RATE_PP:
        print(f'[판정불가] 덜 흔들었다 (pitch {p_pp:.1f}도 < {MIN_PITCH_PP}, '
              f'또는 각속도 {w_pp:.1f} < {MIN_RATE_PP})')
        print('           더 크고 빠르게 다시 해주세요.')
        return 2

    if r is None:
        print('[판정불가] 변화가 없다.')
        return 2

    if r > 0.8:
        print('=' * 68)
        print(' [정상]  각속도 부호가 pitch 와 같은 방향이다.')
        print('         kp_pitch_rate 를 써도 된다 — 제동으로 작동한다.')
        print('=' * 68)
        return 0
    if r < -0.8:
        print('=' * 68)
        print(' [반대!] 각속도 부호가 뒤집혀 있다.')
        print('         이대로 kp_pitch_rate 를 켜면 **제동이 가속이 되어 발산한다.**')
        print('         즉시:  ros2 param set /pid_control_node kp_pitch_rate 0.0')
        print('         고치려면 pid_control_node.cpp 의 rate_err 부호를 뒤집어야 한다.')
        print('=' * 68)
        return 1
    print(f'[판정불가] r={r:+.3f} 이 애매하다 (|r| > 0.8 이어야 한다).')
    print('           roll 을 같이 흔들었거나 너무 느리게 움직였을 수 있다. 다시.')
    return 2

if __name__ == '__main__':
    sys.exit(main())
