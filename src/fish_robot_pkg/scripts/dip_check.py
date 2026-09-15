#!/usr/bin/env python3
"""복각 점검 — 실험 직전에 "나침판 기준이 제대로 잡혔나"를 판정한다 (2026-09-15).

왜 필요한가
    state_estimation_ekf_node 는 **서비스가 켜지는 순간 첫 지자기 샘플의 복각을 기준으로 잡고**,
    그 뒤로는 그 기준에서 ±15도 넘게 벗어난 샘플을 "교란"으로 보고 버린다.
    켤 때 로봇 옆에 쇠붙이·자석이 있으면 기준이 틀리게 잡히고, 그 뒤 깨끗한 곳으로 옮겨도
    **정상 값이 전부 버려진다.** 2026-09-14 20:17 기동이 실제로 그랬다 — 기준 175.8도,
    버려진 값의 중앙값 144.3도(정답 143도). 저널에 ERROR 한 줄만 남고 조용히 진행된다.

무엇을 보나 (서비스를 켠 채로, 로봇을 가만히 둔 상태에서)
    ① 기동 기준 — 이번 기동에서 EKF 가 저널에 남긴 복각 기준값
    ② 지금 자리 — /raw/magnetometer + /raw/imu_6dof 로 직접 잰 복각과 자기장 세기
    ③ EKF 가 지금 나침판을 쓰고 있나 — /filtered/ekf_status[10] 플래그

    복각은 그 지역의 물리 상수(한국 약 143도)라 로봇 자세와 무관하게 일정해야 한다.
    보정·축 변환은 EKF 와 **같은 식**을 쓴다: 칩 축에서 (raw - bias) * scale, 그다음 FLU 재배치.
    축 규약은 magneto_cal.py 한 곳에서 가져온다.

사용
    python3 ~/ros2_ws/src/fish_robot_pkg/scripts/dip_check.py          # 5초 측정
    python3 ~/ros2_ws/src/fish_robot_pkg/scripts/dip_check.py --sec 10
    python3 ~/ros2_ws/src/fish_robot_pkg/scripts/dip_check.py --no-journal   # 저널 없이 지금 자리만

종료 코드: 0 = 진행 가능, 1 = 조치 필요, 2 = 데이터 없음
토픽을 읽기만 한다 — 아무것도 발행하지 않으므로 서비스와 충돌하지 않는다.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from magneto_cal import IMU_TO_FLU, MAG_TO_FLU, dip_of  # noqa: E402  축 규약의 진실원은 magneto_cal.py

CALIB_FILE = os.path.expanduser("~/ros2_ws/config/mag_calib_params.txt")
EKF_DIP_GATE = 15.0     # EKF mag_dip_gate_deg — 기준에서 이만큼 벗어나면 샘플을 버린다
NORMAL_M_UT = 38.0      # 교란 없는 자리의 보정 후 |m| (2026-09-14 21:42 / 22:00 기동 실측 39.1 / 38.2 µT).
                        # AK8963 ASA 미적용이라 실제 지구 자기장(~50 µT)보다 30 % 낮게 읽힌다 (CLAUDE.md 알려진 결함).
STILL_G = 0.03          # |a| 가 중앙값에서 이 이상 벗어나면 움직인 것으로 본다 (magneto_cal.py 와 같은 값)

FLAG_MAG_OK = 0x04
FLAG_MAG_REJ = 0x10
FLAG_MAG_LOST = 0x20


def load_calib(path):
    with open(path) as f:
        v = [float(x) for x in f.read().split()]
    if len(v) < 6:
        raise ValueError(f"보정 파일 형식이 이상합니다 ({len(v)}개 값): {path}")
    return np.array(v[0:3]), np.array(v[3:6])


def journal_reference():
    """이번 서비스 기동에서 EKF 가 잡은 복각 기준. (상태, 기준도, 편차도, 기동시각) 를 돌려준다.
    상태: 'ok' | 'error' | 'none'(아직 없음) | 'inactive' | 'unreadable'"""
    try:
        active = subprocess.run(["systemctl", "is-active", "fish-robot"],
                                capture_output=True, text=True, timeout=10).stdout.strip()
        if active != "active":
            return ("inactive", None, None, None)
        since = subprocess.run(["systemctl", "show", "-p", "ActiveEnterTimestamp", "--value", "fish-robot"],
                               capture_output=True, text=True, timeout=10).stdout.strip()
        out = subprocess.run(["journalctl", "-u", "fish-robot", "--since", since, "-o", "cat", "--no-pager"],
                             capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return ("unreadable", None, None, None)
    return parse_reference(out.splitlines()) + (since,)


def parse_reference(lines):
    """저널 줄들에서 **마지막** 복각 기준을 찾는다 (보정 성공 시 EKF 가 기준을 다시 잡으므로 마지막이 유효).
    (상태, 기준도, 편차도)"""
    ok_re = re.compile(r"복각 기준 확보: ([0-9.]+)도 \(기대 ([0-9.]+)도, 편차 ([0-9.]+)도\)")
    err_re = re.compile(r"복각이 물리적으로 불가능합니다: 실측 ([0-9.]+)도, 기대 ([0-9.]+)도")
    found = ("none", None, None)
    for ln in lines:
        m = ok_re.search(ln)
        if m:
            found = ("ok", float(m.group(1)), float(m.group(3)))
            continue
        m = err_re.search(ln)
        if m:
            found = ("error", float(m.group(1)), abs(float(m.group(1)) - float(m.group(2))))
    return found


def measure(sec, mag_topic, imu_topic, status_topic, bias, scale):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Vector3
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    node = Node("dip_check")
    acc = {"a": None}
    pairs, flags = [], []
    cnt = {"imu": 0, "mag": 0}

    def on_imu(msg):
        cnt["imu"] += 1
        a = msg.linear_acceleration
        acc["a"] = IMU_TO_FLU(a.x, a.y, a.z)

    def on_mag(msg):
        cnt["mag"] += 1
        if acc["a"] is None or not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            return
        c = (np.array([msg.x, msg.y, msg.z]) - bias) * scale          # EKF 와 같은 순서: 칩 축 보정
        pairs.append((MAG_TO_FLU(c[0], c[1], c[2]), acc["a"]))           # 그다음 FLU 재배치

    def on_status(msg):
        if len(msg.data) > 10:
            flags.append(int(msg.data[10]))

    node.create_subscription(Imu, imu_topic, on_imu, 50)
    node.create_subscription(Vector3, mag_topic, on_mag, 50)
    node.create_subscription(Float32MultiArray, status_topic, on_status, 10)

    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    return pairs, flags, cnt


def main():
    p = argparse.ArgumentParser(description="실험 전 복각(나침판 기준) 점검")
    p.add_argument("--sec", type=float, default=5.0, help="측정 시간(초). 로봇은 가만히 둘 것")
    p.add_argument("--dip", type=float, default=143.0, help="그 지역 복각 (한국 143)")
    p.add_argument("--tol", type=float, default=10.0, help="143도에서 이 이내면 정상 (좋은 보정 실측 1~7도)")
    p.add_argument("--no-journal", action="store_true", help="저널을 읽지 않고 지금 자리만 본다")
    p.add_argument("--ref", type=float, help="기동 기준 복각을 직접 지정 (시험·오프라인용, 저널 대신)")
    p.add_argument("--calib", default=CALIB_FILE)
    p.add_argument("--mag-topic", default="/raw/magnetometer")
    p.add_argument("--imu-topic", default="/raw/imu_6dof")
    p.add_argument("--status-topic", default="/filtered/ekf_status")
    args = p.parse_args()

    bar = "=" * 74
    print(bar)
    print(f" 복각 점검  (기대 {args.dip:.0f}도, 정상 범위 ±{args.tol:.0f}도, 측정 {args.sec:.0f}초 — 로봇을 가만히 두세요)")
    print(bar)

    try:
        bias, scale = load_calib(args.calib)
    except (OSError, ValueError) as e:
        print(f" ❌ 보정 파일을 읽을 수 없습니다: {e}")
        return 2
    print(f" 보정 파일  bias({bias[0]:.2f} {bias[1]:.2f} {bias[2]:.2f})  scale({scale[0]:.4f} {scale[1]:.4f} {scale[2]:.4f})")

    # ── ① 기동 기준 ──
    if args.ref is not None:
        ref_state, ref, ref_err = "ok" if abs(args.ref - args.dip) <= 30 else "error", args.ref, abs(args.ref - args.dip)
        print(f" ① 기동 기준  {ref:.1f}도 (직접 지정)")
    elif args.no_journal:
        ref_state, ref, ref_err = "skip", None, None
        print(" ① 기동 기준  (건너뜀: --no-journal)")
    else:
        ref_state, ref, ref_err, since = journal_reference()
        if ref_state == "inactive":
            print(" ① 기동 기준  서비스가 꺼져 있습니다 (sudo systemctl start fish-robot)")
        elif ref_state == "unreadable":
            print(" ① 기동 기준  저널을 읽을 수 없습니다 (--no-journal 로 지금 자리만 볼 수 있음)")
        elif ref_state == "none":
            print(f" ① 기동 기준  아직 없음 — 기동({since}) 뒤 EKF 가 지자기 기준을 잡지 않았습니다")
            print("              (EKF 는 IMU 가 들어와 초기화된 뒤 첫 지자기 샘플로 기준을 잡는다 — 아래 ② 를 볼 것)")
        else:
            mark = "✅" if ref_err <= args.tol else ("⚠" if ref_state == "ok" else "❌")
            print(f" ① 기동 기준  {mark} {ref:.1f}도 (편차 {ref_err:.1f}도)   기동 {since}")

    # ── ② 지금 자리 ──
    pairs, flags, cnt = measure(args.sec, args.mag_topic, args.imu_topic, args.status_topic, bias, scale)
    if len(pairs) < 20:
        print(f" ② 지금 자리  ❌ 데이터 부족 — IMU {cnt['imu']}개, 지자기 {cnt['mag']}개 ({args.sec:.0f}초 동안, 정상이면 각각 약 {int(args.sec * 100)}개)")
        if cnt["imu"] == 0 and cnt["mag"] > 0:
            print("              지자기는 들어오는데 IMU 가 없습니다 = nRF52840 쪽이 안 보냅니다.")
            print("              로봇(nRF) 전원과 Pi-nRF UART 선을 확인하세요. IMU 가 없으면 EKF 가 초기화를 못 해")
            print("              복각 기준도 안 잡힙니다. nRF 를 켠 뒤에는 서비스 재시작 없이 다시 이 점검을 돌리면 됩니다.")
        elif cnt["mag"] == 0 and cnt["imu"] > 0:
            print("              IMU 는 들어오는데 지자기가 없습니다 = AK8963 무응답입니다.")
            print("              J4 커넥터를 다시 꽂고 서비스를 재시작하세요 (지자기는 기동 때 한 번만 찾는다, CLAUDE.md §8).")
        elif cnt["imu"] == 0 and cnt["mag"] == 0:
            print("              둘 다 없습니다. 서비스가 떠 있는지, 같은 ROS_DOMAIN_ID 인지, 로봇 전원을 확인하세요.")
        else:
            print("              둘 다 들어오지만 너무 적습니다. --sec 를 늘려 다시 돌리세요.")
        print(bar)
        return 2
    M = np.array([m for m, _ in pairs], dtype=float)
    A = np.array([a for _, a in pairs], dtype=float)
    an = np.linalg.norm(A, axis=1)
    still = np.abs(an - np.median(an)) < STILL_G
    still_pct = 100.0 * still.mean()
    use = still if still.sum() >= 20 else np.ones(len(A), dtype=bool)
    d = dip_of(M[use], A[use])
    live, live_sd = float(d.mean()), float(d.std())
    mn = float(np.linalg.norm(M[use], axis=1).mean())
    live_err = abs(live - args.dip)
    m_ratio = mn / NORMAL_M_UT
    live_ok = live_err <= args.tol and 0.75 <= m_ratio <= 1.25
    mark = "✅" if live_ok else "❌"
    print(f" ② 지금 자리  {mark} 복각 {live:.1f} ± {live_sd:.1f}도 (편차 {live_err:.1f}도)   "
          f"자기장 {mn:.1f} µT (평소 약 {NORMAL_M_UT:.0f})   샘플 {len(pairs)}개")
    if not 0.75 <= m_ratio <= 1.25:
        print(f"              ⚠ 자기장 세기가 평소의 {m_ratio:.2f}배 — 근처에 쇠붙이·자석·전원선이 있다는 신호입니다")
    moving = still_pct < 80.0
    if moving:
        print(f"              ⚠ 측정 중 움직였습니다 (정지 {still_pct:.0f}%). 가만히 두고 다시 돌리세요")

    # ── ③ EKF 가 나침판을 쓰고 있나 ──
    if flags:
        ok_pct = 100.0 * sum(1 for f in flags if f & FLAG_MAG_OK) / len(flags)
        rej_pct = 100.0 * sum(1 for f in flags if f & FLAG_MAG_REJ) / len(flags)
        lost = any(f & FLAG_MAG_LOST for f in flags)
        mark = "✅" if ok_pct >= 50 else "❌"
        extra = "  ⚠ 지자기 두절(6축) 상태가 있었습니다" if lost else ""
        print(f" ③ EKF 사용   {mark} 나침판 반영 {ok_pct:.0f}% / 버림 {rej_pct:.0f}%   (플래그 마지막 {flags[-1]}){extra}")
    else:
        ok_pct, rej_pct = None, None
        print(" ③ EKF 사용   (상태 토픽 없음 — EKF 가 안 떠 있거나 다른 도메인)")
    if ref is not None:
        gap = abs(live - ref)
        note = ("EKF 복각 게이트는 통과 (세기 게이트로 버려질 수는 있음)" if gap <= EKF_DIP_GATE
                else "EKF 복각 게이트에서 버려지는 차이")
        print(f"              지금 복각과 기동 기준의 차이 {gap:.1f}도 — {note} (±{EKF_DIP_GATE:.0f}도)")

    # ── 판정 ──
    print(bar)
    ref_good = ref is not None and ref_err <= args.tol
    ekf_ok = ok_pct is None or ok_pct >= 50
    if moving:
        print(" ⚠ 판정 보류: 로봇을 가만히 둔 채로 다시 실행하세요.")
        code = 1
    elif live_ok and (ref_good or ref_state == "skip") and not ekf_ok:
        print(" ⚠ 기준과 지금 자리는 정상인데 EKF 가 나침판 값을 버리고 있습니다.")
        print("    방금 로봇을 들거나 돌렸다면 흔한 일입니다. 가만히 두고 30초 뒤 다시 점검하세요.")
        print("    가만히 둔 채로도 계속 버리면 자세마다 복각이 달라지는 보정 문제입니다. 로봇을 다른 자세")
        print("    (옆으로 눕히기 등)로 놓고 이 점검을 반복해, 자세 사이 복각 차이가 15도를 넘으면 지자기 보정을 다시 하세요.")
        code = 1
    elif live_ok and (ref_good or ref_state == "skip"):
        if ref_state == "skip":
            print(" ✅ 지금 자리는 깨끗합니다. (기동 기준은 보지 않았습니다 — 저널 확인을 권장)")
        else:
            print(" ✅ 실험 진행 가능 — 기준도 맞고, 지금 자리도 깨끗합니다.")
        code = 0
    elif live_ok and ref is not None and not ref_good:
        print(" ❌ 기준이 교란된 자리에서 잡혔습니다. 지금 자리는 깨끗하니 여기서 서비스를 재시작하세요:")
        print("      sudo systemctl restart fish-robot    (대기압 영점 180초·자이로 정지 10초가 다시 돈다)")
        print("    그대로 두면 EKF 가 정상 값을 계속 버리고 회전 센서만으로 방향을 계산합니다.")
        code = 1
    elif not live_ok and ref_good:
        print(" ❌ 기준은 맞지만 지금 자리가 교란돼 있습니다. 쇠붙이·자석에서 1 m 떨어진 곳으로 옮겨 다시 보세요.")
        print("    이 자리에 있는 동안 EKF 는 나침판 값을 버리고 회전 센서로만 방향을 잡습니다 (위험하진 않음).")
        code = 1
    elif not live_ok:
        print(" ❌ 지금 자리도 기준도 맞지 않습니다. 깨끗한 곳으로 옮겨 이 점검을 다시 하고,")
        print("    거기서도 복각이 벗어나면 그 자리에서 서비스를 재시작하세요. 그래도 안 되면 지자기 보정(btn2 3초)을 다시.")
        code = 1
    else:
        print(" ⚠ 기동 기준을 확인하지 못했습니다. 위 ① 줄을 보고 조치하세요.")
        code = 1
    print(bar)
    return code


if __name__ == "__main__":
    sys.exit(main())
