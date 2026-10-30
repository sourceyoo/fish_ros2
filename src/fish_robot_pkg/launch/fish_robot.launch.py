import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 0. DDS 탐색 범위를 이 호스트 안으로 제한 (2026-10-30)
        #    Jazzy 기본값은 SUBNET 이다 (/opt/ros/jazzy/share/ros_environment/
        #    environment/1.ros_discovery.sh 가 set-if-unset 으로 심는다. 서비스
        #    유닛에 Environment= 가 없어도 setup.bash 를 source 하므로 들어온다).
        #    SUBNET + wlan0 활성이면 같은 Pi 안의 노드끼리도 공유메모리와 UDP
        #    **두 경로로 같은 메시지가 각각 전달**된다. 받는 쪽은 중복인지 모르니
        #    둘 다 처리한다. 실측 (2026-10-30, /raw/imu_6dof 전용 구독자):
        #      SUBNET    : 199.5 Hz,  dt<=0  45.6 %,  dt 중앙 7.39 ms
        #      LOCALHOST : 100.0 Hz,  dt<=0   0.0 %,  dt 중앙 10.00 ms
        #    양방향 2회 재현 + 경로 수 0/1/2 용량-반응까지 확인했다.
        #
        #    피해가 두 가지다.
        #    (1) 두 경로의 지연이 달라 과거 스탬프가 나중에 도착한다 →
        #        state_estimation_ekf_node.cpp:739 의 dt<=0 "단절" 분기가 떠서
        #        공분산을 계속 부풀린다. 10-30 13:08 기동분에서 누적 45만 회.
        #    (2) /filtered/attitude 가 200 Hz 가 되고, pid_control_node 는 타이머
        #        없이 그 콜백에서 PID 를 돌므로(CLAUDE.md 규칙 5) /motor/output 과
        #        nRF UART 패킷도 두 배가 된다. 발행자가 둘일 때의 피해와 같다.
        #
        #    rcl/discovery_options.h:53 이 "향후 버전에서 LOCALHOST 를 기본값으로
        #    할 의도" 라고 적고 있으니 상류가 가는 방향과도 같다.
        #    ※ 영향 범위: 이 Pi 밖에 ROS2 를 설치해 네트워크 건너편에서 토픽을
        #      직접 구독하는 방식만 막힌다. SSH 로 들어와서 쓰는 ros2 topic echo /
        #      ros2 param set / rqt / journalctl 은 전부 그대로 된다(명령이 이 호스트
        #      안에서 돌기 때문). 외부 모니터링이 필요해지면 SUBNET 으로 되돌리는
        #      대신 ROS_DOMAIN_ID 를 기본 0 에서 바꿔 다른 기기와의 충돌만 막아라.
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),

        # 1. UART 브릿지 노드 (6축 및 RC 데이터 허브)
        Node(
            package='fish_robot_pkg',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 2. I2C 드라이버 노드 (지자기·3채널 압력 + MPU9250 6축 감시 /raw/imu_mpu9250, 2026-09-16)
        Node(
            package='fish_robot_pkg',
            executable='i2c_driver_node',
            name='i2c_driver_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 3. PID 제어 노드 (목표 각도 추종 PID 제어기 및 모터 믹싱)
        Node(
            package='fish_robot_pkg',
            executable='pid_control_node',
            name='pid_control_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 4. RPM 드라이버 노드 (하드웨어 외부 인터럽트 독립 수집가)
        Node(
            package='fish_robot_pkg',
            executable='rpm_driver_node',
            name='rpm_driver_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 5. 데이터 로거 노드 (예전 C 펌웨어 규격 매칭 일치형 실시간 영구 파일 로거)
        Node(
            package='fish_robot_pkg',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 6. 자동 시나리오 노드 (자율 주행 시나리오 궤적 생성)
        Node(
            package='fish_robot_pkg',
            executable='auto_scenario_node',
            name='auto_scenario_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 7. EKF 자세 추정 노드 — 제어 경로 추정기 (구 state_estimation_node 완전 대체, 2026-08-17)
        #    구 state_estimation_node 블록은 여기서 제거됐다. 같이 띄우면
        #    /filtered/attitude 발행자가 둘이 되어 pid_control_node 가 200Hz로 돌고
        #    (dt 생략 PID라 게인이 통째로 틀어진다) 서로 다른 추정치가 번갈아 들어가며,
        #    /calibrate_mag 서비스도 중복 광고가 된다. 수동 실행으로도 함께 띄우지 말 것.
        #    확인: ros2 topic info /filtered/attitude --verbose | grep -c PUBLISHER  -> 1
        #    롤백: 이 파일의 대체 커밋을 revert 하고 fish-robot 서비스 재시작.
        #
        #    출력: /filtered/attitude (제어 — ±5000 모드 인코딩 + yaw 영점),
        #          /filtered/attitude_ekf (순수값, 비교/진단), /filtered/ekf_status,
        #          /sensor/pressure_calibrated, /CH_IMU, /CH_MEGNET
        #    서비스: /calibrate_mag (magneto_cal.py 실행/조기 종료 토글 — min/max 폐기)
        #
        #    ※ respawn=True — 제어 경로에 들어갔으므로 다른 노드와 동일하게 되살린다.
        #      (관측 전용이던 시절의 respawn=False 근거는 대체와 함께 소멸)
        #
        #    use_mag=True 근거 (2026-08-07 실측):
        #      - 6축 yaw 드리프트 +0.155도/분 (합격 기준 2.0) 통과
        #      - 9축에서 교란 중 기울기 오차가 Mahony의 1/2.8 (1.36도 -> 0.49도)
        #      - 지자기 상실 시 500ms 만에 6축으로 떨어지고 자세 계단은 0.09도.
        #        복구 때도 roll/pitch가 0.01도밖에 안 움직인다 (Mahony는 2.7도)
        Node(
            package='fish_robot_pkg',
            executable='state_estimation_ekf_node',
            name='state_estimation_ekf_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_accel': True,
                'use_mag': True,
                'mag_yaw_sigma': 8.0,
            }]
        ),

        # 8. 수심/속도 추정 노드 — 압력 3채널에서 수심(현재)과 전진 속도(다음 단계)
        #    출력: /filtered/hydro [depth, speed, q, q_raw, dP_hydro, static, pitch, flags, + 대기창 영점 7칸(2026-09-16)]
        #    서비스: /hydro/zero_atm (대기압 영점 재포착)
        #
        #    ※ 관측 전용이다 — 어떤 제어 루프도 아직 /filtered/hydro 를 구독하지 않는다.
        #    ※ 수심은 좌우 정압 포트의 산술평균이라 **자세와 무관하게 정확**하다
        #      (압력이 깊이에 선형이므로 두 점의 평균 = 기하학적 중점의 압력).
        #      그래서 IMU가 빠져 있어도, 지자기 보정이 없어도 수심은 나온다.
        #    ※ 포트 역할(front/left/right)은 여기가 아니라 config/port_map.txt 에서
        #      읽는다. 센서 지문(PROM) 대조와 한 곳에 묶어두기 위해서다.
        Node(
            package='fish_robot_pkg',
            executable='hydro_estimator_node',
            name='hydro_estimator_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'water_density': 997.0,      # 담수 25도. 해수는 1025
                'atm_warmup_sec': 180.0,     # EKF의 PRESSURE_WARMUP_SEC 과 같은 값
            }]
        ),
    ])
