// =============================================================================
//  pid_control_node.cpp
//
//  역할 : 자세 오차를 PID로 보정해 최종 모터 4채널 출력을 만드는 제어기
//
//   [입력] /filtered/attitude  (Vector3)    <- 현재 자세 + 모드 플래그
//          /filtered/angular_rate (Vector3) <- 각속도 [도/초, FLU, 바이어스 제거]
//                                            pitch 종속 제어용 (2026-10-30). 기본 꺼짐
//          /rc/command         (Quaternion) <- 조종기 목표값 (수동 모드)
//          /auto/command       (Quaternion) <- 시나리오 목표값 (자동 모드).
//                                            z = 서보 고정 플래그 (2026-09-16, 아래 참조)
//          /sensor/tail_rpm    (Int32)      <- 꼬리 RPM (현재는 수집만)
//   [출력] /motor/output       (UInt16MultiArray[4]) -> uart_bridge_node
//          /pid/debug          (Float32MultiArray[4]) -> [kp_pitch, pitch_open_loop, 목표pitch, 목표roll]
//                                 (2026-10-15) data_logger_node 가 CSV 69~72열로 쓴다. **진단 전용** —
//                                 제어 경로는 이 토픽을 읽지 않는다. 게인과 개루프 스위치는 실행 중
//                                 바뀌므로(ros2 param set, 즉시 반영) "그 순간 무엇으로 제어했나"가
//                                 로그에 함께 남아야 사후 대조가 된다. 목표값도 같이 싣는 이유는
//                                 AUTO 시나리오 목표(/auto/command)와 수동 스틱 목표가 같은 열에
//                                 모여야 모드와 무관하게 "지시한 각도"를 한 열에서 읽을 수 있기 때문이다.
//
//  실행 시점 : 타이머가 아니라 /filtered/attitude 수신 콜백에서 PID를 돌린다.
//              즉 제어 주기 = 자세 발행 주기 = IMU 100Hz에 자동으로 동기화된다.
//
//  출력 채널 배치 : [0]=좌 서보, [1]=우 서보, [2]=요 서보, [3]=꼬리 BLDC 스로틀
//
//  ※ 요 제어 없음 (2026-09-14, 운용자 결정): 요 서보 [2] 는 항상 1500(중앙) 고정이다.
//     조종기 요 스틱(/rc/command z)은 읽지 않고, 헤딩 오차로 서보를 움직이는 요 PID 도
//     없다. 좌우 지느러미 믹싱은 원래 롤·피치만 쓰므로 영향이 없다. 되살릴 때는 git
//     이력에서 kp_yaw 파라미터 시절을 볼 것 — 그때 요 오차가 ±180 경계를 감싸지 않던
//     결함도 같이 고쳐야 한다.
//
//  ※ 서보 고정 플래그 — /auto/command z (2026-09-16): 요가 비우고 간 z 칸을 재활용한다.
//     **요 목표가 아니다.** AUTO 에서 z >= 0.5 면 PID 를 통째로 건너뛰고
//     [1500, 1500, 1500, 1000] 을 그대로 낸다 (auto_scenario_node 의 `static` 시나리오).
//     "추력 0, 롤·피치 0" 만으로는 가만히 있지 않기 때문이다 — 목표가 0 이어도 로봇이
//     기울면 오차가 생기고 PID 가 지느러미를 움직인다. 구동 없이 센서만 재려면 출력
//     자체를 끊어야 한다. MANUAL 에서는 무시한다(조종기 경로는 z 를 안 쓴다).
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <cmath>

// 서보 동작 범위: 혼 기준 약 ±20° (2026-08-26 확정).
//   서보는 HS-5086WP — 표준 RC 신호 ±400us 에 총 91° 를 도는 기종이라
//   0.114°/us 이고, ±20° 는 ±175us, 즉 1325~1675 다.
// nRF 펌웨어 MotorControl.cpp 는 1250~1750 으로 더 넓다. 규칙은 "동일"이 아니라
// **"여기 창이 nRF 창 안에 포함될 것"** 이다 — 여기가 더 좁으면 nRF 클램프에는
// 절대 걸리지 않으므로 안전하고, 여기를 nRF 보다 넓히는 것만 금지다 (N5).
#define SERVO_MIN_US 1325
#define SERVO_MAX_US 1675

// =============================================================================
//  pitch 종속(각속도) 제어 — 2026-10-30
// =============================================================================
//  왜: 지금 pitch 는 각도 오차에 P만 쓴다 (ki/kd 전부 0). 목표에 가까워질 때
//  **제동이 없어** 관성으로 지나치고 되돌아오는 진동이 구조적으로 남는다.
//  교과서적 답은 kd 지만 이 구현은 dt 를 곱하지 않으므로(규칙 5) D 를 켜는 순간
//  게인 의미가 깨진다. 그래서 자이로로 각속도를 **직접 측정**해 쓴다 — 미분이
//  없으니 dt 문제를 피해 간다.
//
//  구조:
//    목표각속도 = kp_pitch_angle * 각도오차        [도/초]
//    목표각속도 = clamp(목표각속도, ±pitch_rate_max)
//    각속도오차 = 목표각속도 − 측정각속도(LPF)      [도/초]
//    u_pitch   = kp_pitch * (각도오차 + kp_pitch_rate * 각속도오차)
//
//  **단위를 반드시 보라**: kp_pitch 는 us/도 다 (8.75 = 175us/20°, 아래 생성자
//  주석). 그래서 각속도 항을 kp_pitch **안쪽**에 넣어 "등가 각도" 로 환산한다.
//  바깥에 더하면 us 와 도/초 를 섞게 되고 게인 스케일이 통째로 틀어진다.
//  kp_pitch_rate 의 단위는 도/(도/초) = 초다.
//
//  **기본값 kp_pitch_rate = 0 → 거동이 지금과 수학적으로 완전히 동일하다.**
//  코드만 들어가고 출력은 1 us 도 안 바뀐다. 물에서 ros2 param set 으로 조금씩
//  올리며 튜닝하고, 이상하면 0 으로 되돌리면 즉시 원복이다 (재빌드 불필요).
//
//  실측 근거 (10-19~10-26 주행 로그 96개 RUN 구간, pitch 각속도 도/초):
//      static 0.5(p50)/7.1(p90)   straight 4.8/12.8
//      dive   5.8/19.0            dive_hold 12.9/35.3
//    - pitch_rate_max 40: 가장 격한 dive_hold p90 35.3 바로 위. 실제 거동을
//      막지 않으면서 그 위를 자른다. 처음 추정 30 은 dive_hold 를 깎았다.
//    - kp_pitch_angle 4.0: 역수 0.25초 = "틀어진 각도를 0.25초에 지우려는 속도".
//      dive 실측과 맞는다. 처음 추정 2.0(0.5초)은 실제보다 느렸다.
//    - pitch_rate_lpf_tau 0.05: straight 는 목표 0° 인데도 4.8 도/초가 나온다
//      (static 의 10배) — 자세가 틀어진 게 아니라 **꼬리 박자에 몸통이 떠는
//      것**이다. 필터 없이 각속도 항에 넣으면 지느러미가 그 떨림에 맞춰 흔들려
//      서보 수명만 깎는다. 제어 시간상수(0.25초)의 1/5 이라 제어 대역은 거의
//      건드리지 않는다. 키우면 매끄러워지지만 각속도가 늦게 와 제동이 늦은
//      제동이 되고, 그건 진동의 원인이다.
//
//  ※ 부호: 이 로봇은 **양수 pitch = 기수 아래**로 항공 관례와 반대다
//    (CLAUDE.md Known gaps). 각속도 부호가 반대면 **제동이 가속이 된다.**
//    kp_pitch_rate 를 0 보다 올리기 전에 반드시 벤치에서 손으로 기울여
//    확인할 것 — PWM 이나 부호 추론으로 판정하지 말 것 (믹싱 주석의 2026-09-04
//    교훈과 같은 종류의 함정이다).
//    **확인됨 (2026-10-31, scripts/rate_sign_check.py)**: 기수 상하 흔들기 1,780
//    표본, pitch 진폭 64.9°, 각속도 진폭 78.3 °/s, 상관 r = +0.992 → 각속도 y 는
//    pitch 와 같은 부호다. 제동으로 작동한다. 그래서 기본값을 0.0 → 0.1 로 올렸다
//    (운용자 결정). IMU 를 다시 장착하면 이 측정은 무효 — 스크립트를 다시 돌릴 것.
//  ※ roll 은 건드리지 않았다. 종속 제어는 pitch 에만 넣는다 — 실측 근거가
//    pitch 각속도뿐이고, roll 까지 동시에 바꾸면 물에서 원인 분리가 안 된다.
// =============================================================================
#define PITCH_RATE_DEFAULT_MAX 40.0f    // [도/초] 목표 각속도 상한

class PidControlNode : public rclcpp::Node {
public:
    PidControlNode() : Node("pid_control_node"),
                       // PID 게인: 현재 P만 사용 중 (I/D는 0으로 비활성)
                       // kp 8.75 = 175us / 20°: 오차 1° 당 혼이 약 1° 꺾이고(0.114°/us
                       // 역수), 오차 20° 에서 창 끝(±175us)에 닿는다. 이전 3.5 는
                       // 포화에 오차 71° 가 필요해 벤치에서 "찔끔" 움직였다.
                       // ※ 벤치용 정적 배율이다 — 물에서 출렁이면 낮추는 게 §5 튜닝.
                       kp_r_(8.75f), ki_r_(0.0f), kd_r_(0.0f),
                       kp_p_(8.75f), ki_p_(0.0f), kd_p_(0.0f),
                       // pitch 종속(각속도) 제어. kp_p_rate_ 0 = 꺼짐 = 기존 거동.
                       kp_p_angle_(4.0f), kp_p_rate_(0.1f),
                       rate_max_(PITCH_RATE_DEFAULT_MAX), rate_lpf_tau_(0.05f),
                       rate_max_age_(0.05),
                       i_limit_(300.0f),                       // 적분 와인드업 상한 (ki=0인 지금은 무효)
                       // 서보 방향: 우측 지느러미는 몸 반대편에 **거울상으로 장착**돼
                       // 같은 PWM 증가가 물리적으로 반대 회전이 된다. 실측(2026-08-26):
                       // 피치 명령에 좌우가 반대로 돌고(=물리 롤), 롤 명령에 같이 돌았다
                       // (=물리 피치) — 명령의 자세↔서보 상관은 정상(−0.93/−0.96)이었으므로
                       // 원인은 믹싱이 아니라 장착 방향이다.
                       //
                       // 2026-09-04 벤치에서 PID·스틱을 빼고 /motor/output 에 고정 PWM 을
                       // 직접 꽂아 확인했다: **좌우에 같은 PWM(1675)을 주면 지느러미가
                       // 서로 반대로 벌어진다** = 거울상 장착이 맞다. 따라서 피치(지느러미
                       // 같은 방향)를 내려면 PWM 은 반대여야 하고, 우측만 반전이 옳다.
                       //
                       // 같은 날 이 값을 false 로 잠깐 뒤집었다가 AUTO 시나리오 2(피치 다운)
                       // 에서 양쪽이 반대로 벌어져 되돌렸다. **수동에서 보이던 이상은 이
                       // 값이 아니라 스틱 롤/피치 스왑이 원인이었다** (swap_rc_rp_ 참조) —
                       // 수동만 보고 이 값을 만지면 AUTO 가 깨진다. 판정은 AUTO 로 할 것.
                       servo_rev_l_(false), servo_rev_r_(true),
                       // 축 부호. 피치는 IMU 규약이 항공과 반대(양수 pitch = 기수 아래)라
                       // 뒤집힐 소지가 상시 있다 — 벤치에서 지느러미를 보고 확정할 것.
                       invert_pitch_(false), invert_roll_(false),
                       pitch_open_loop_(false),
                       // 조종기 짐벌 배선 스왑을 파이에서 우회하던 스위치. 조종기 펌웨어를
                       // 고쳤으므로(2026-09-04) 기본값은 false 다 — 켜 두면 두 번 뒤집혀 원위치.
                       swap_rc_rp_(false),
                       // 스틱 부호. 조종기가 스틱 다운에 음수를 보내는데 이 IMU 규약은
                       // 양수가 기수 아래라, 피치만 뒤집어야 다운이 다운이 된다.
                       rc_inv_roll_(false), rc_inv_pitch_(true),
                       // 스틱 데드밴드: 실측(45초 정지)에서 roll 스틱이 중립인데 −12까지
                       // 방황했고(트림 오프셋+잡음), 스로틀이 0→28 스파이크를 냈다.
                       rc_deadband_(15), throttle_deadband_(40),
                       err_sum_roll_(0.0f), err_sum_pitch_(0.0f),
                       prev_err_roll_(0.0f), prev_err_pitch_(0.0f),
                       robot_roll_(0.0f), robot_pitch_(0.0f),
                       // rc_throttle_를 -9999(두절 센티넬)로 시작 -> 첫 조종기 패킷이
                       // 도착하기 전까지는 페일세이프 상태로 묶어둔다.
                       rc_roll_(0), rc_pitch_(0), rc_throttle_(-9999),
                       auto_roll_(0), auto_pitch_(0), auto_throttle_(1000), auto_hold_(false),
                       is_auto_mode_(false),
                       current_rpm_(0),
                       failsafe_timeout_(5.0),                 // 조종기 무수신 허용 시간(초)
                       last_valid_rc_time_(this->now())
    {
        // ── 게인을 ROS 파라미터로 노출 ─────────────────────────────────
        // 물속 튜닝 때 빌드+재시작(약 1분) 없이 즉시 바꾸기 위해서다:
        //   ros2 param set /pid_control_node kp_pitch 10.0
        // 기본값은 위 초기화 목록의 값을 그대로 쓴다 — 진실원은 한 곳이다.
        kp_r_ = (float)this->declare_parameter<double>("kp_roll",  (double)kp_r_);
        ki_r_ = (float)this->declare_parameter<double>("ki_roll",  (double)ki_r_);
        kd_r_ = (float)this->declare_parameter<double>("kd_roll",  (double)kd_r_);
        kp_p_ = (float)this->declare_parameter<double>("kp_pitch", (double)kp_p_);
        ki_p_ = (float)this->declare_parameter<double>("ki_pitch", (double)ki_p_);
        kd_p_ = (float)this->declare_parameter<double>("kd_pitch", (double)kd_p_);
        i_limit_ = (float)this->declare_parameter<double>("i_limit", (double)i_limit_);
        servo_rev_l_ = this->declare_parameter<bool>("servo_reverse_left",  servo_rev_l_);
        servo_rev_r_ = this->declare_parameter<bool>("servo_reverse_right", servo_rev_r_);
        // 축 부호 — 서보 반전과 목적이 다르다. 믹싱 주석의 1)/2) 구분을 볼 것.
        invert_pitch_ = this->declare_parameter<bool>("invert_pitch", invert_pitch_);
        invert_roll_  = this->declare_parameter<bool>("invert_roll",  invert_roll_);
        // pitch 개루프 스위치 (2026-10-14, 판별 실험용). true + AUTO 이면 pitch 핀을 측정 pitch 없이
        // kp_pitch × 목표 pitch 로만 낸다. roll 은 그대로 피드백. 기본 false = 기존 동작과 동일.
        pitch_open_loop_ = this->declare_parameter<bool>("pitch_open_loop", pitch_open_loop_);

        // pitch 종속(각속도) 제어 — 전부 live-settable. 파일 상단 설계 주석 참조.
        // kp_pitch_rate 기본 0 = 각속도 항 무효 = 지금과 동일한 거동.
        kp_p_angle_   = (float)this->declare_parameter<double>("kp_pitch_angle",    (double)kp_p_angle_);
        kp_p_rate_    = (float)this->declare_parameter<double>("kp_pitch_rate",     (double)kp_p_rate_);
        rate_max_     = (float)this->declare_parameter<double>("pitch_rate_max",    (double)rate_max_);
        rate_lpf_tau_ = (float)this->declare_parameter<double>("pitch_rate_lpf_tau",(double)rate_lpf_tau_);
        rate_max_age_ =        this->declare_parameter<double>("pitch_rate_max_age", rate_max_age_);
        swap_rc_rp_   = this->declare_parameter<bool>("swap_rc_roll_pitch", swap_rc_rp_);
        rc_inv_roll_  = this->declare_parameter<bool>("invert_rc_roll",  rc_inv_roll_);
        rc_inv_pitch_ = this->declare_parameter<bool>("invert_rc_pitch", rc_inv_pitch_);
        rc_deadband_       = (int)this->declare_parameter<int64_t>("rc_deadband",       rc_deadband_);
        throttle_deadband_ = (int)this->declare_parameter<int64_t>("throttle_deadband", throttle_deadband_);

        // 런타임 변경 반영. 변경은 WARN 으로 남긴다 — 물속 튜닝에서 "몇 시에 어떤
        // 게인이었나"가 저널에 남아야 CSV 와 대조할 수 있다.
        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&PidControlNode::on_param_change, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 게인: roll %.2f/%.2f/%.2f  pitch %.2f/%.2f/%.2f (kp/ki/kd) ===",
            kp_r_, ki_r_, kd_r_, kp_p_, ki_p_, kd_p_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 요 제어 없음: 요 서보 1500 고정, 요 스틱·시나리오 요 목표 무시 (2026-09-14) ===");
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 서보 반전: 좌 %d / 우 %d  스틱 데드밴드: rp %d, 스로틀 %d ===",
            servo_rev_l_, servo_rev_r_, rc_deadband_, throttle_deadband_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 축 부호 반전: 피치 %d / 롤 %d  (서보 반전과 다른 것 — 축 하나만 뒤집는다) ===",
            invert_pitch_, invert_roll_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 수동 스틱 롤<->피치 스왑: %d  (AUTO 경로에는 영향 없음) ===", swap_rc_rp_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 수동 스틱 부호 반전: 롤 %d / 피치 %d  (AUTO 는 규약이 달라 제외) ===",
            rc_inv_roll_, rc_inv_pitch_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] pitch 개루프 스위치: %d (기본 0 = 피드백) ===", pitch_open_loop_);
        RCLCPP_INFO(this->get_logger(),
            "=== [PID Control] 런타임 튜닝: ros2 param set /pid_control_node kp_pitch <값> ===");

        // 종속 제어 상태를 기동 때 명시한다. 꺼져 있다는 사실이 로그에 남아야
        // "코드는 들어갔는데 왜 거동이 같나" 를 journal 에서 바로 확인할 수 있다.
        if (kp_p_rate_ == 0.0f) {
            RCLCPP_WARN(this->get_logger(),
                "[PID Control] pitch 종속(각속도) 제어 **꺼짐** (kp_pitch_rate=0) "
                "-> 각도 P 단독, 기존 거동과 동일. 켜기: ros2 param set "
                "/pid_control_node kp_pitch_rate <값>  (부호 벤치 확인 선행 필수)");
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[PID Control] pitch 종속(각속도) 제어 켜짐: kp_rate=%.3f s, "
                "kp_angle=%.2f 1/s, rate_max=%.1f 도/초, LPF tau=%.3f s, max_age=%.0f ms",
                kp_p_rate_, kp_p_angle_, rate_max_, rate_lpf_tau_, rate_max_age_ * 1000.0);
        }

        motor_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/motor/output", 10);

        // 진단 전용. 숫자 네 개라 커스텀 메시지를 만들지 않는 관례를 따른다.
        debug_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid/debug", 10);

        // 이 구독이 제어 루프의 심장이다 (콜백 안에서 PID 연산 + 발행까지 수행)
        // 각속도 구독 (pitch 종속 제어용). depth 1 — 최신값만 쓸모가 있다.
        // 낡은 각속도로 제동을 걸면 진동의 원인이 되므로 쌓아둘 이유가 없다.
        angular_rate_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/angular_rate", 1,
            std::bind(&PidControlNode::angular_rate_callback, this, std::placeholders::_1));

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&PidControlNode::attitude_callback, this, std::placeholders::_1));

        rc_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/rc/command", 10, std::bind(&PidControlNode::rc_command_callback, this, std::placeholders::_1));

        auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/auto/command", 10, std::bind(&PidControlNode::auto_command_callback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor/tail_rpm", 10, std::bind(&PidControlNode::rpm_callback, this, std::placeholders::_1));
    }

private:
    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // state_estimation이 roll에 실어 보낸 ±5000 오프셋을 벗겨내 실제 각도를 복원하고,
        // 오프셋 유무 자체를 자동/수동 모드 플래그로 사용한다.
        // (roll 실제 범위는 ±180도라 2500을 넘으면 오프셋이 실린 것이 확실하다)
        if (msg->x > 2500.0f) {
            robot_roll_ = msg->x - 5000.0f;
            is_auto_mode_ = true;
        } else if (msg->x < -2500.0f) {
            robot_roll_ = msg->x + 5000.0f;
            is_auto_mode_ = true;
        } else {
            robot_roll_ = msg->x;
            is_auto_mode_ = false;
        }

        robot_pitch_ = msg->y;
        // msg->z(yaw)는 쓰지 않는다 — 요 제어 없음 (2026-09-14)
        calculate_pid_control();   // 자세가 갱신될 때마다 즉시 제어 1주기 실행
    }

    void rc_command_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        // -9999는 nRF52840이 보내는 "조종기 링크 끊김" 센티넬이다.
        // 이 값이 오면 저장된 목표값을 갱신하지 않고 타임스탬프도 찍지 않는다.
        // -> 마지막 정상 수신 시각이 멈추므로 아래 failsafe_timeout_ 검사에 걸린다.
        int16_t incoming_throttle = static_cast<int16_t>(msg->w);
        if (incoming_throttle != -9999) {
            // **스틱 축 스왑 우회 스위치. 지금은 꺼져 있는 것이 정상이다.**
            //
            // 2026-09-04 실측: 조종기 스틱을 가로로만 흔들면 /rc/command 의 y(피치)가,
            // 세로로만 흔들면 x(롤)가 움직였다 — 조종기 짐벌 배선이 뒤바뀌어 있었다
            // (세로축 포텐쇼미터가 roll_pin 에). 그래서 피치 스틱이 롤 명령이 되어
            // "피치를 주는데 지느러미가 롤처럼 반대로 움직인다" 가 나왔다.
            //
            // **근본 수정은 조종기 펌웨어에서 했다** (Input_Manager.cpp 의 roll_pin/
            // pitch_pin #define 교체, 2026-09-04). 그래서 이 스위치는 false 다.
            // 조종기를 고친 채로 이걸 켜면 두 번 뒤집혀 다시 망가진다.
            //
            // ※ 이건 **수동 경로만**의 문제였다. AUTO 는 /auto/command 로 따로 오므로
            //   영향이 없다 — 그래서 서보 반전으로 수동을 맞추면 AUTO 가 틀어지고,
            //   AUTO 를 맞추면 수동이 틀어지는 모순이 생겼다. 두 문제가 겹쳐 서로를
            //   가리고 있었던 것이다(다른 하나는 servo_rev_r_ 주석 참조).
            // ※ /rc/command 원시값은 건드리지 않는다 — 로깅·진단이 원본을 봐야 한다.
            //   그래서 이 스위치를 켜면 CSV 의 RC_R/RC_P 열은 뒤바뀐 채로 남는다.
            if (swap_rc_rp_) {
                rc_roll_  = static_cast<int16_t>(msg->y);
                rc_pitch_ = static_cast<int16_t>(msg->x);
            } else {
                rc_roll_  = static_cast<int16_t>(msg->x);
                rc_pitch_ = static_cast<int16_t>(msg->y);
            }
            // msg->z(요 스틱)는 읽지 않는다 — 요 제어 없음 (2026-09-14).
            // 원본은 data_logger 가 CSV RC_Y 열에 그대로 남긴다.
            rc_throttle_ = incoming_throttle;
            last_valid_rc_time_ = this->now();
        }
    }

    void auto_command_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        auto_roll_ = static_cast<int16_t>(msg->x);
        auto_pitch_ = static_cast<int16_t>(msg->y);
        // msg->z 는 요 목표가 아니라 **서보 고정 플래그**다 (2026-09-16, 파일 머리말 참조).
        // 요 제어는 2026-09-14 에 없앴고, 비어 버린 이 칸을 재활용한 것이다.
        auto_hold_ = (msg->z >= 0.5);
        auto_throttle_ = static_cast<int16_t>(msg->w);
    }

    // 꼬리 BLDC RPM 수신 (추후 surge speed 제어에서 사용 예정)
    void rpm_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_rpm_ = msg->data;
    }

    // 런타임 게인 변경. ki 를 바꾸면 그 축의 적분 누산을 0으로 리셋한다 —
    // 누산(err_sum_)은 ki=0 인 동안에도 계속 쌓여 ±i_limit 에 붙어 있으므로,
    // 리셋 없이 ki 를 켜면 그 순간 i_limit×ki 만큼 출력이 점프한다.
    // ※ dt 를 게인에 흡수한 구현이라(아래 주의 참조) I/D 게인은 100Hz 기준 값이다.
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &prm : params) {
            const std::string &n = prm.get_name();
            if (n == "servo_reverse_left")  { servo_rev_l_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 서보 반전 변경: 좌 = %d", servo_rev_l_); continue; }
            if (n == "servo_reverse_right") { servo_rev_r_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 서보 반전 변경: 우 = %d", servo_rev_r_); continue; }
            if (n == "invert_pitch")        { invert_pitch_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 축 부호 변경: 피치 반전 = %d", invert_pitch_); continue; }
            if (n == "invert_roll")         { invert_roll_  = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 축 부호 변경: 롤 반전 = %d",  invert_roll_);  continue; }
            if (n == "swap_rc_roll_pitch")  { swap_rc_rp_   = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 수동 스틱 롤<->피치 스왑 = %d", swap_rc_rp_); continue; }
            if (n == "invert_rc_roll")      { rc_inv_roll_  = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 수동 스틱 부호: 롤 반전 = %d",  rc_inv_roll_);  continue; }
            if (n == "invert_rc_pitch")     { rc_inv_pitch_ = prm.as_bool(); RCLCPP_WARN(this->get_logger(), "[PID Control] 수동 스틱 부호: 피치 반전 = %d", rc_inv_pitch_); continue; }
            if (n == "pitch_open_loop") {
                pitch_open_loop_ = prm.as_bool();
                err_sum_pitch_ = 0.0f; prev_err_pitch_ = 0.0f;   // 켜고 끌 때 적분·미분 상태 비움 (복귀 시 튐 방지)
                RCLCPP_WARN(this->get_logger(), "[PID Control] pitch 개루프 스위치 변경: %d", pitch_open_loop_);
                continue;
            }
            if (n == "rc_deadband")       { rc_deadband_       = (int)prm.as_int(); RCLCPP_WARN(this->get_logger(), "[PID Control] rc_deadband = %d", rc_deadband_); continue; }
            if (n == "throttle_deadband") { throttle_deadband_ = (int)prm.as_int(); RCLCPP_WARN(this->get_logger(), "[PID Control] throttle_deadband = %d", throttle_deadband_); continue; }
            const float v = (float)prm.as_double();
            if      (n == "kp_roll")  kp_r_ = v;
            else if (n == "ki_roll")  { ki_r_ = v; err_sum_roll_  = 0.0f; }
            else if (n == "kd_roll")  kd_r_ = v;
            else if (n == "kp_pitch") kp_p_ = v;
            else if (n == "ki_pitch") { ki_p_ = v; err_sum_pitch_ = 0.0f; }
            else if (n == "kd_pitch") kd_p_ = v;
            else if (n == "i_limit")  i_limit_ = v;
            else if (n == "kp_pitch_angle")     kp_p_angle_ = v;
            else if (n == "kp_pitch_rate")      kp_p_rate_  = v;
            else if (n == "pitch_rate_max")     rate_max_   = v;
            else if (n == "pitch_rate_lpf_tau") rate_lpf_tau_ = v;
            else if (n == "pitch_rate_max_age") rate_max_age_ = (double)v;
            else continue;
            RCLCPP_WARN(this->get_logger(), "[PID Control] 게인 변경: %s = %.3f", n.c_str(), v);
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    }

    // 각속도 수신 (pitch 종속 제어용, 2026-10-30)
    //  - y 축만 쓴다 (pitch). FLU 이고 단위는 도/초, 바이어스는 EKF 가 이미 뺐다.
    //  - LPF 는 **여기서** 건다. 꼬리 박자 떨림(straight 4.8 도/초, static 의 10배)을
    //    제어에 그대로 넣지 않기 위한 것이다. 파일 상단 설계 주석 참조.
    //  - dt 는 발행 간격을 실측해서 쓴다. attitude 와 같은 콜백에서 나오므로
    //    100Hz 가 정상이지만, 고정 0.01 로 가정하면 레이트가 흔들릴 때 시간상수가
    //    조용히 틀어진다 (2026-10-30 의 DDS 중복 전달이 바로 그런 사례였다).
    void angular_rate_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        const float raw = (float)msg->y;
        if (std::isnan(raw)) return;          // NaN 은 LPF 상태를 영구 오염시킨다

        const rclcpp::Time now = this->now();
        float dt = 0.01f;
        if (rate_initialized_) {
            const double d = (now - last_rate_time_).seconds();
            // 비정상 간격(음수/정체/과대)은 dt 로 쓰지 않고 공칭값으로 둔다.
            if (d > 0.0 && d < 0.5) dt = (float)d;
        }
        last_rate_time_ = now;

        if (!rate_initialized_) {
            pitch_rate_ = raw;                // 첫 샘플은 필터를 통과시킨다
            rate_initialized_ = true;
        } else if (rate_lpf_tau_ <= 0.0f) {
            pitch_rate_ = raw;                // tau<=0 = 필터 끔
        } else {
            // 1차 IIR. alpha = dt/(tau+dt) 형태라 dt 가 변해도 시간상수가 유지된다.
            const float a = dt / (rate_lpf_tau_ + dt);
            pitch_rate_ += a * (raw - pitch_rate_);
        }
    }

    // 서보 출력 클램프: 기구부가 물리적으로 낼 수 있는 각도 밖으로 나가지 않게 막는다.
    uint16_t constrain_servo(float val) {
        // NaN은 모든 비교가 false라 클램프를 통과하고 (uint16_t)NaN=UB(보통 0)가 된다.
        // 명시적으로 걸러 중립(1500)으로 폴백한다 (N6).
        if (std::isnan(val)) return 1500;
        if (val < static_cast<float>(SERVO_MIN_US)) return SERVO_MIN_US;
        if (val > static_cast<float>(SERVO_MAX_US)) return SERVO_MAX_US;
        return static_cast<uint16_t>(val);
    }

    // BLDC 스로틀 클램프: ESC 표준 PWM 범위(1000~2000us)
    uint16_t constrain_pwm(float val) {
        if (std::isnan(val)) return 1000;   // NaN → 최소 스로틀로 안전 폴백 (N6)
        if (val < 1000.0f) return 1000;
        if (val > 2000.0f) return 2000;
        return static_cast<uint16_t>(val);
    }

    void calculate_pid_control() {
        double elapsed_since_last_cmd = (this->now() - last_valid_rc_time_).seconds();

        // ---- [페일세이프] 수동 모드에서 조종기가 끊기면 즉시 중립 출력 후 종료 ----
        // 두 가지 경우를 모두 잡는다:
        //   (1) rc_throttle_ == -9999  : nRF가 명시적으로 "링크 끊김"을 통보
        //   (2) 타임아웃               : nRF 자체가 죽어 /rc/command가 아예 안 오는 경우
        // ※ 자동 모드(is_auto_mode_)에서는 이 검사를 건너뛴다. 조종기 없이 도는 것이
        //    자율 주행의 정상 동작이기 때문이다.
        if (!is_auto_mode_ && (rc_throttle_ == -9999 || elapsed_since_last_cmd > failsafe_timeout_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[PID Control] 수동 모드 중 조종기 통신 두절 (Timeout: %.1f초). 모터 정지.", elapsed_since_last_cmd);
            auto motor_output_msg = std_msgs::msg::UInt16MultiArray();
            motor_output_msg.data = {1500, 1500, 1500, 1000};   // 서보 3개 중립 + 추진 정지
            motor_pub_->publish(motor_output_msg);
            return;
        }

        auto motor_output_msg = std_msgs::msg::UInt16MultiArray();
        motor_output_msg.data.resize(4);

        float target_roll, target_pitch;
        uint16_t active_throttle;

        // ---- 목표값 소스 선택 (자동이면 시나리오, 수동이면 조종기) ----
        if (is_auto_mode_) {
            // 0.1도 단위 정수 -> 실제 각도(도)
            target_roll = auto_roll_ * 0.1f;
            target_pitch = auto_pitch_ * 0.1f;
            active_throttle = constrain_pwm(auto_throttle_);   // 시나리오는 이미 PWM 단위로 준다
        } else {
            // 스틱 데드밴드 — **MANUAL 분기에만** 둔다. AUTO 경로(auto_*)에 새면 시나리오
            // 명령이 깎이고, uart_bridge 에서 깎으면 data_logger 의 원시 스틱 기록이
            // 사라져 오늘 같은 잡음 진단이 불가능해진다. 여기가 유일하게 옳은 자리다.
            // roll/pitch 는 소프트(재중심): 경계 불연속 없이 중심 부근 미세 트림을
            // 지킨다. 최대 목표각이 25.5°→24.0° 로 주는 것은 무시한다.
            auto soft_db = [this](int16_t st) -> int16_t {
                if (st >  rc_deadband_) return (int16_t)(st - rc_deadband_);
                if (st < -rc_deadband_) return (int16_t)(st + rc_deadband_);
                return 0;
            };
            // **스틱 부호 — MANUAL 에만 건다. AUTO 와 규약이 다르기 때문이다.**
            //
            // 이 IMU 는 **양수 pitch = 기수 아래**다(항공 규약 반대, 실측 확정).
            // AUTO 시나리오는 그 규약대로 쓰여 있다 — dive 가 pitch = +angle 이다.
            // 반면 조종기는 스틱을 내리면 −256 이 나온다. 그대로 목표각으로 쓰면
            // "다운을 줬는데 기수가 올라간다" 가 된다 (2026-09-04 실기 확인).
            //
            // 그래서 뒤집는 자리는 여기, 수동 분기다. 축 부호(invert_pitch_)나
            // 서보 반전으로 고치면 **AUTO 까지 같이 뒤집혀** 멀쩡한 시나리오가 깨진다.
            // 근본 수정은 조종기의 map() 방향이고, 그쪽을 고치면 이 값을 false 로.
            target_roll  = (rc_inv_roll_  ? -1.0f : 1.0f) * soft_db(rc_roll_)  * 0.1f;
            target_pitch = (rc_inv_pitch_ ? -1.0f : 1.0f) * soft_db(rc_pitch_) * 0.1f;
            // 스로틀은 하드 데드밴드: "의도적 입력 전까지 ESC 는 정확히 1000" 이라는
            // 안전 속성이 연속성보다 중요하다. −9999 센티넬은 위 페일세이프가 이미
            // 걸렀으므로 여기 도달한 rc_throttle_ 은 실제 스틱 값이다.
            int16_t thr = (rc_throttle_ < throttle_deadband_) ? 0 : rc_throttle_;
            // 조종기 스로틀 원시 범위(0~1171)를 ESC PWM 범위(1000~2000)로 선형 사상
            int16_t mapped_throttle = static_cast<int16_t>(thr * (1000.0f / 1171.0f));
            active_throttle = constrain_pwm(1000 + mapped_throttle);
        }

        // ---- [완전 정지] 시나리오가 서보 고정을 요청하면 PID 를 통째로 건너뛴다 ----
        // 목표를 0 으로 주는 것과 다르다: 목표가 0 이어도 로봇이 기울면 오차가 생겨
        // 지느러미가 움직인다. 여기서는 출력 자체를 중립으로 못박아 구동을 끊는다.
        // 적분·미분 상태도 함께 지운다 — 정지 구간의 오차가 쌓여 있다가 다음 주행
        // 시작 순간에 한꺼번에 튀어나오면(와인드업) 그게 곧 첫 동작이 되어 버린다.
        if (is_auto_mode_ && auto_hold_) {
            err_sum_roll_ = 0.0f; err_sum_pitch_ = 0.0f;
            prev_err_roll_ = 0.0f; prev_err_pitch_ = 0.0f;
            motor_output_msg.data[0] = 1500;
            motor_output_msg.data[1] = 1500;
            motor_output_msg.data[2] = 1500;
            motor_output_msg.data[3] = 1000;   // 추력도 끊는다 — 플래그의 뜻이 "완전 정지"다
            motor_pub_->publish(motor_output_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[PID Control] 완전 정지 — 서보 1500 고정, 추력 1000 (PID 건너뜀, 센서만 측정)");
            return;
        }

        // PID 연산 구역
        // 주의: dt를 곱하지 않는 형태다. 제어 주기가 100Hz로 일정하다는 전제 아래
        //       dt를 게인에 흡수시킨 구현이며, I항은 단순 누산 / D항은 단순 차분이다.
        float err_roll = target_roll - robot_roll_;
        float err_pitch = target_pitch - robot_pitch_;

        // 자세값이 NaN이면(asinf 이상·센서 결함 등) 여기서 적분/미분 상태(err_sum_, prev_err_)에
        // NaN이 스며 재시작 전까지 영구 오염되는 것을 막는다. 상태를 건드리지 않고 안전
        // 출력(서보 중립, 스로틀 유지) 후 반환한다 (N6). 자세 복구 시 깨끗하게 재개된다.
        if (std::isnan(err_roll) || std::isnan(err_pitch)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[PID Control] 자세값 NaN 감지. 이번 주기 서보 중립 출력 (적분 상태 보존).");
            motor_output_msg.data[0] = 1500;
            motor_output_msg.data[1] = 1500;
            motor_output_msg.data[2] = 1500;
            motor_output_msg.data[3] = active_throttle;
            motor_pub_->publish(motor_output_msg);
            return;
        }

        // I항: 오차 누산
        err_sum_roll_ += err_roll;
        err_sum_pitch_ += err_pitch;

        // 안티 와인드업: 누산값을 ±i_limit_로 잘라 적분항이 폭주하지 않게 한다.
        // (제어가 오래 포화 상태에 머물 때 적분이 무한정 커져 복귀가 늦어지는 현상 방지)
        if (err_sum_roll_ > i_limit_) err_sum_roll_ = i_limit_;
        else if (err_sum_roll_ < -i_limit_) err_sum_roll_ = -i_limit_;

        if (err_sum_pitch_ > i_limit_) err_sum_pitch_ = i_limit_;
        else if (err_sum_pitch_ < -i_limit_) err_sum_pitch_ = -i_limit_;

        // D항: 직전 주기 대비 오차 변화량
        float d_roll = err_roll - prev_err_roll_;
        float d_pitch = err_pitch - prev_err_pitch_;

        prev_err_roll_ = err_roll;
        prev_err_pitch_ = err_pitch;

        // 축별 제어 출력 u = P + I + D
        float u_roll = (kp_r_ * err_roll) + (ki_r_ * err_sum_roll_) + (kd_r_ * d_roll);
        // pitch 개루프(판별 실험용): AUTO 이고 스위치가 켜져 있으면 측정 pitch 를 쓰지 않는다.
        // 핀 = kp_pitch × 목표 pitch (dive 15° → 8.75×15 = 131us → M1 1631 / M2 1368 고정).
        // 적분·미분 상태는 비워 둔다 — 스위치를 끈 순간 쌓인 값이 튀어나오지 않게.
        const bool pitch_ol = is_auto_mode_ && pitch_open_loop_;
        if (pitch_ol) { err_sum_pitch_ = 0.0f; prev_err_pitch_ = 0.0f; }

        // ---- pitch 종속(각속도) 제어 ----
        // 각도오차에 "각속도오차 × kp_pitch_rate" 를 **등가 각도로** 더해 kp_pitch
        // 한 번만 곱한다. kp_pitch 가 us/도 이므로 바깥에서 더하면 단위가 섞인다
        // (파일 상단 설계 주석).
        //
        // 각속도를 쓰지 않는 조건 세 가지 — 어느 하나라도 걸리면 각도 제어로
        // 자동 복귀한다. **0 을 쓰지 않고 항 자체를 빼는** 이유: 각속도 0 은
        // "정지" 를 뜻하므로, 실제로는 돌고 있는데 0 을 넣으면 제동이 아니라
        // 엉뚱한 가속 명령이 된다.
        //   1) kp_pitch_rate == 0        : 기능 꺼짐 (기본값)
        //   2) 아직 한 번도 못 받음      : 초기화 전
        //   3) 받은 값이 너무 낡음       : EKF 멎음/토픽 끊김. 낡은 각속도로 거는
        //      제동은 이미 지나간 움직임에 반응하는 것이라 진동을 키운다.
        float rate_err = 0.0f;
        bool  rate_used = false;
        if (kp_p_rate_ != 0.0f && rate_initialized_) {
            const double age = (this->now() - last_rate_time_).seconds();
            if (age >= 0.0 && age <= rate_max_age_) {
                // 목표 각속도 = 각도오차 × kp_pitch_angle, ±rate_max 로 제한.
                // 제한이 없으면 큰 각도에서 로봇이 못 내는 속도를 명령해 지느러미가
                // 끝에 붙어 있는 동안 제어가 사실상 먹지 않는다.
                float rate_cmd = kp_p_angle_ * err_pitch;
                if (rate_cmd >  rate_max_) rate_cmd =  rate_max_;
                else if (rate_cmd < -rate_max_) rate_cmd = -rate_max_;
                rate_err  = rate_cmd - pitch_rate_;
                rate_used = true;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[PID Control] 각속도 %.0f ms 지연 -> 이번 주기 각도 제어로 복귀", age * 1000.0);
            }
        }

        // 개루프일 때는 종속 항도 쓰지 않는다 — 개루프의 정의가 "측정 pitch 를
        // 쓰지 않는다" 이고, 각속도는 측정값이다.
        float u_pitch = pitch_ol
            ? (kp_p_ * target_pitch)
            : (kp_p_ * (err_pitch + kp_p_rate_ * rate_err))
              + (ki_p_ * err_sum_pitch_) + (kd_p_ * d_pitch);

        // ---- 모터 믹싱 ----
        // 좌우 가슴지느러미 서보를 차동 구동한다:
        //   두 서보를 같은 방향으로 움직이면 -> 피치(상승/하강)
        //   서로 반대 방향으로 움직이면     -> 롤(좌우 기울기)
        //
        // **부호가 두 종류이고 고치는 자리가 다르다 — 섞으면 무한히 헛돈다.**
        //
        //   1) 축 부호 (invert_pitch_ / invert_roll_) — 그 축 **하나만** 뒤집는다.
        //      "피치는 반대인데 롤은 맞다" 는 여기서 고친다.
        //   2) 서보 반전 (servo_rev_*) — 그 **서보 하나만** 뒤집는다.
        //      "우측 지느러미만 반대로 간다" 는 여기서 고친다.
        //
        //   양쪽 서보를 동시에 반전하는 것은 **축 부호를 둘 다 뒤집는 것과 같다** —
        //   피치를 고치려고 좌우를 함께 반전하면 롤이 딸려서 뒤집힌다 (2026-09-04
        //   실기에서 실제로 이 함정에 걸렸다). 축 하나만 문제면 반드시 1) 을 쓴다.
        //
        // 2026-08-26 에는 우측만 반전(true)했는데 **물리 확인 없이 PWM 부호로
        // 추론한** 방향이었고 증상이 그대로였다. 2026-09-04 벤치에서 PID·스틱을
        // 빼고 /motor/output 에 고정 PWM 을 직접 꽂아 눈으로 확인해 정정했다.
        // 부호 판정은 반드시 지느러미를 보고 한다 — PWM 부호로 추론하지 말 것.
        float p_term = invert_pitch_ ? -u_pitch : u_pitch;
        float r_term = invert_roll_  ? -u_roll  : u_roll;
        float d_left  = p_term + r_term;
        float d_right = p_term - r_term;
        if (servo_rev_l_) d_left  = -d_left;
        if (servo_rev_r_) d_right = -d_right;
        motor_output_msg.data[0] = constrain_servo(1500 + d_left);    // 좌 서보
        motor_output_msg.data[1] = constrain_servo(1500 + d_right);   // 우 서보
        motor_output_msg.data[2] = 1500;                              // 요 서보: 중앙 고정 (요 제어 없음, 2026-09-14)
        motor_output_msg.data[3] = active_throttle;                            // 꼬리 BLDC 추진

        if (is_auto_mode_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[AUTO] Target: R=%.1f, P=%.1f | Out: M1=%d, M2=%d",
                target_roll, target_pitch, motor_output_msg.data[0], motor_output_msg.data[1]);
        }

        motor_pub_->publish(motor_output_msg);

        // 진단: 그 순간의 게인·개루프 스위치·목표값. 제어에는 쓰이지 않는다.
        // **페일세이프·완전정지로 조기 return 한 경로에서는 발행되지 않는다** — 그 구간의
        // CSV 열에는 직전 값이 남고, 그게 맞다(그때는 PID 가 돌지 않았다는 뜻이므로
        // State/Mode 열로 구분된다).
        std_msgs::msg::Float32MultiArray dbg;
        // 0~3 은 **순서를 바꾸지 말 것** — data_logger_node 가 CSV 69~72열로 읽는다
        // (size()<4 검사 후 인덱스 0~3만 읽으므로 뒤에 덧붙이는 것은 무해하다).
        // 4~6 은 종속 제어 진단 (2026-10-30): 각속도를 실제로 썼는지, 그때 측정
        // 각속도와 각속도오차가 얼마였는지. 튜닝 중 "제동이 걸렸나" 를 로그로
        // 확인하려면 이 셋이 필요하다.
        // 7 은 그 순간의 kp_pitch_rate 자체다 (2026-10-31). 이게 **물에서 ros2 param set
        // 으로 올려가며 튜닝하는 값**이므로, kp_pitch 를 0 번에 실은 것과 같은 이유로
        // 함께 남겨야 한다 — 없으면 여러 값으로 시험한 로그에서 어느 구간이 어느
        // 게인이었는지 사후에 갈릴 수 없다. rate_used(4) 는 "쓰였나" 일 뿐 값이 아니다.
        dbg.data = { kp_p_, pitch_ol ? 1.0f : 0.0f, target_pitch, target_roll,
                     rate_used ? 1.0f : 0.0f, pitch_rate_, rate_err,
                     kp_p_rate_ };
        debug_pub_->publish(dbg);
    }

    // PID 게인 (r=roll, p=pitch). 요 게인은 없다 — 요 제어 없음 (2026-09-14)
    float kp_r_, ki_r_, kd_r_;
    float kp_p_, ki_p_, kd_p_;
    // pitch 종속(각속도) 제어 게인 (2026-10-30). 선언 순서 = 생성자 초기화 순서 (-Wreorder, 2026-10-31)
    float kp_p_angle_;            // 각도오차 -> 목표각속도 [1/s]
    float kp_p_rate_;             // 각속도오차 -> 등가각도 [s]. **0 = 꺼짐**
    float rate_max_;              // 목표각속도 상한 [도/초]
    float rate_lpf_tau_;          // 각속도 LPF 시간상수 [초]. <=0 이면 필터 끔
    double rate_max_age_;         // 이보다 낡은 각속도는 쓰지 않는다 [초]
    float i_limit_;               // 적분 누산 한계 (안티 와인드업)
    bool servo_rev_l_, servo_rev_r_;                         // 채널별 장착 방향 반전 (요 서보는 고정이라 없음)
    bool invert_pitch_, invert_roll_;                        // 축별 부호 반전 (믹싱 이전)
    bool pitch_open_loop_;                                   // pitch 개루프 (판별 실험용, 2026-10-14)
    bool swap_rc_rp_;                                        // 수동 스틱 롤<->피치 스왑
    bool rc_inv_roll_, rc_inv_pitch_;                        // 수동 스틱 부호 반전 (AUTO 제외)
    int  rc_deadband_, throttle_deadband_;                   // 스틱 데드밴드 [카운트]
    float err_sum_roll_, err_sum_pitch_;                     // I항 누산 상태
    float prev_err_roll_, prev_err_pitch_;                   // D항 계산용 직전 오차
    float robot_roll_, robot_pitch_;                         // 현재 자세(측정값)

    // pitch 종속(각속도) 제어 상태 (2026-10-30). 게인은 위 PID 게인 옆에 선언됨
    float pitch_rate_ = 0.0f;     // LPF 통과 pitch 각속도 [도/초]
    bool  rate_initialized_ = false;
    rclcpp::Time last_rate_time_{0, 0, RCL_ROS_TIME};

    int16_t rc_roll_, rc_pitch_, rc_throttle_;                   // 수동 목표값 (0.1도 / 스틱 raw)
    int16_t auto_roll_, auto_pitch_, auto_throttle_;             // 자동 목표값 (0.1도 / PWM)
    bool    auto_hold_;                                          // /auto/command z — 서보 고정(완전 정지)

    bool is_auto_mode_;

    int32_t current_rpm_;  // 최신 꼬리 BLDC RPM (추후 surge speed 제어용)

    double failsafe_timeout_;          // 조종기 무수신 허용 시간(초)
    rclcpp::Time last_valid_rc_time_;  // 마지막으로 정상 조종기 패킷을 받은 시각

    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angular_rate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr auto_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidControlNode>());
    rclcpp::shutdown();
    return 0;
}
