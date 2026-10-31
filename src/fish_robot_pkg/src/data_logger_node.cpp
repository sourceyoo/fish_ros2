// =============================================================================
//  data_logger_node.cpp
//
//  역할 : 시스템 전 토픽을 한 곳에서 구독해 100Hz로 CSV 한 줄씩 기록하는 블랙박스
//
//   [입력] /filtered/attitude          (Vector3)             자세 + 모드 플래그
//          /sensor/pressure_calibrated (Float32MultiArray[3]) 영점 보정된 압력
//          /sensor/pressure_raw        (Float32MultiArray[6]) 원시 절대압[0..2] + 온도[3..5]
//          /rc/command                 (Quaternion)          조종기 스틱
//          /rc/status                  (Int32MultiArray[5])  버튼/배터리/RSSI
//          /motor/output               (UInt16MultiArray[4]) 최종 모터 출력
//          /sensor/tail_rpm            (Int32)               꼬리 RPM
//          /raw/imu_6dof               (Imu)                  원시 6축 — g, 도/초 그대로
//          /raw/magnetometer           (Vector3)              원시 지자기[µT], 보정 전 칩 축
//          /filtered/ekf_status        (Float32MultiArray[12]) [0..2] 자이로 바이어스, [10] 플래그
//          /filtered/hydro             (Float32MultiArray[15]) 수심[m]·속도[m/s]·동압·자세보정 + 대기창 영점
//          /auto/state                 (Int32)                100*시나리오ID + 상태 (auto_scenario_node).
//                                                                 둘로 나눠 State·Scenario 열에 적는다
//          /raw/imu_mpu9250            (Float32MultiArray[7]) MPU9250 6축[g, 도/초, nRF IMU 칩 축으로 회전됨] + 칩 온도[°C] — 감시용 2번째 IMU
//          /auto/run_info              (Float32MultiArray[4]) [회차, 추력%, 각도, 주행초] — 진입 때 잠근 값
//          /pid/debug                  (Float32MultiArray[4]) [kp_pitch, 개루프, 목표pitch, 목표roll]
//   [출력] ~/ros2_ws/log_csv/fish_log_YYYYMMDD_HHMMSS.csv
//
//  ── 절대 시각 (2026-10-07) ─────────────────────────────────────────────────
//  Time(s) 는 **로거 기동 기준 경과 초**라 그 자체로는 다른 장비의 기록과 맞출 수 없다.
//  예인 장치처럼 로봇 밖에서 따로 돌아가는 기록(파일명에 20261005_144706 식으로 시각이
//  박힌다)과 같은 시간선에 올리려면 절대 시각이 필요하다. 파일명으로 역산할 수도 있지만
//  초 단위라 10ms 행을 가릴 수 없고, 200MB 회전 뒤에는 Time(s) 가 이어지므로 두 번째
//  파일부터는 파일명이 기준이 아니게 되어 더 어긋난다.
//
//  그래서 **마지막 열에 Epoch(s) 를 둔다** — UTC 기준 유닉스 시각(1970-01-01부터의 초),
//  소수점 3자리(ms). 기존 열은 번호도 의미도 그대로이므로 옛 분석 스크립트가 깨지지 않는다.
//    · 다른 장비와 맞추기: 양쪽을 Epoch 로 바꿔 빼면 끝난다 (시간대·서머타임 무관)
//    · 사람이 읽기: date -d @1760000000  또는  파이썬 datetime.fromtimestamp()
//    · Time(s) 와의 차 = 로거 기동 시각. 파일 회전을 넘어도 일정하다
//  ※ 벽시계(system_clock)라 NTP 가 시각을 당기면 **뒤로 뛸 수 있다**. 그래서 구간 길이·
//    주기 계산은 지금까지처럼 **Time(s)(단조 증가)로** 하고, Epoch 는 **다른 기록과
//    맞추는 용도로만** 쓴다. 두 열을 다 남기는 이유가 이것이다.
//
//  설계 : 콜백은 값을 멤버 변수에 "저장만" 하고, 10ms 타이머가 그 순간의
//         최신 스냅샷을 한 줄로 기록한다. 토픽마다 주기가 달라도(압력 11Hz,
//         자세 100Hz) 시간축이 일정한 표가 만들어진다.
//
//  원시값 3종을 상시 기록하는 이유(2026-08-18): 물속 운용은 재현이 안 된다.
//  융합 결과만 남으면 자세 이상이 "센서 탓인지 필터 탓인지" 사후 규명이 불가능하다.
//
//  ※ 크기: 100Hz x 73컬럼 ≈ 시간당 150MB (63컬럼 131MB, 55컬럼 110MB, 47컬럼 99MB). 파일당 200MB(약 2.0시간)에서 새 파일로
//     회전하며, Time(s) 축은 회전을 넘어 이어진다(기동 시각 기준 유지). 오래된
//     파일은 지우지 않는다 — log_csv/에는 실험 CSV와 지자기 보정 파일이 같이
//     살므로 자동 삭제는 위험하다. 총량 관리는 사람 몫.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <cmath>
#include <limits>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

class DataLoggerNode : public rclcpp::Node {
public:
    DataLoggerNode() : Node("data_logger_node"), fp_(nullptr) {
        RCLCPP_INFO(this->get_logger(), "=== [Data Logger] 누적 카운트 및 동기화 로깅 대기 중 ===");

        if (init_log_file() < 0) {
            RCLCPP_ERROR(this->get_logger(), "CSV 파일 시스템 초기화 실패!");
            return;
        }

        // [추가/수정] 버튼 누적 카운트 및 상태 전이 감지용 변수 초기화
        btn1_cumulative_count_ = 0;
        btn1_hold_counter_ = 0;
        prev_btn1_ = 0;
        btn1_long_processed_ = false;
        is_auto_mode_ = false;

        // 각 컬럼의 초기값. 해당 센서가 아직 안 붙은 구간은 이 값이 그대로 기록된다.
        mode_ = 0;
        roll_ = 0.0f; pitch_ = 0.0f; yaw_ = 0.0f;
        rc_t_ = 0; rc_r_ = 0; rc_p_ = 0; rc_y_ = 0;
        m1_ = 1500; m2_ = 1500; m3_ = 1500; m4_ = 1000;   // 서보 중립 + 추진 정지
        rpm_ = 0;
        p0_cal_ = 0.0f; p1_cal_ = 0.0f; p2_cal_ = 0.0f;
        t0_ = 0.0f; t1_ = 0.0f; t2_ = 0.0f;
        // 원시 절대압은 0.0이 아니라 NaN으로 시작한다. 대기압은 ~1013mbar라 0.0이
        // 물리적으로 불가능하긴 하지만, 드라이버가 죽은 채널을 NaN으로 표시하므로
        // (i2c_driver_node.cpp:282-296) "아직 안 받음"도 같은 표기로 두어야
        // 분석 스크립트가 한 가지 규칙(isnan)만 알면 되게 한다.
        {
            const float nan_f = std::numeric_limits<float>::quiet_NaN();
            p0_raw_ = nan_f; p1_raw_ = nan_f; p2_raw_ = nan_f;
        }
        rssi_ = 0;
        vbat1_ = 0;   // 센티볼트. 조종기 미수신 구간에는 0 이 찍힌다
        ax_ = 0.0f; ay_ = 0.0f; az_ = 0.0f;             // 원시 가속도 [g]
        gx_ = 0.0f; gy_ = 0.0f; gz_ = 0.0f;             // 원시 자이로 [도/초]
        mx_ = 0.0f; my_ = 0.0f; mz_ = 0.0f;             // 원시 지자기 [µT]
        bias_x_ = 0.0f; bias_y_ = 0.0f; bias_z_ = 0.0f; // EKF 자이로 바이어스 추정 [도/초]
        ekf_flags_ = 0;
        // hydro 열도 NaN 으로 시작한다. Depth=0.0, Speed=0.0 은 완벽하게 정상인
        // 값이라, 0 초기화는 "노드 미가동"과 "수면에 정지"를 구별 불가능하게 만든다.
        {
            const float nf = std::numeric_limits<float>::quiet_NaN();
            depth_ = nf; speed_ = nf; q_dyn_ = nf; q_raw_ = nf;
            p_hydro_ = nf; hydro_pitch_ = nf;
            q_zero_ = nf; zero_t0_ = nf; zero_t1_ = nf; q_dyn_wz_ = nf; speed_wz_ = nf;
        }
        hydro_seq_ = 0; hydro_flags_ = 0;
        zero_src_ = 0; still_ = 0; auto_state_ = 0; scenario_id_ = 0;
        // 65~72열도 NaN/0 으로 시작한다 — 아직 발행자가 안 붙은 구간을 구별하기 위해
        {
            const float nf = std::numeric_limits<float>::quiet_NaN();
            run_id_ = 0; thr_pct_ = nf; angle_deg_ = nf; run_sec_ = nf;
            kp_pitch_ = nf; pitch_ol_ = 0; target_pitch_ = nf; target_roll_ = nf;
            kp_rate_ = nf; rate_used_ = 0;                     // 74~77열 (2026-10-31)
            rate_meas_ = nf; rate_err_ = nf;
        }
        // 2번째 IMU 도 NaN 으로 시작 — 미수신과 0 을 구별
        for (int i = 0; i < 7; i++) mpu_[i] = std::numeric_limits<float>::quiet_NaN();

        received_attitude_ = false;
        received_pressure_ = false;
        is_logging_active_ = false;

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&DataLoggerNode::attitude_callback, this, std::placeholders::_1));

        pressure_cal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_calibrated", 10, std::bind(&DataLoggerNode::pressure_cal_callback, this, std::placeholders::_1));

        // 원시 압력([0..2] 절대압 mbar)과 원시 온도([3..5] C)를 함께 기록한다.
        // 원시 압력은 수심·속도(피토 차압)를 사후에 다시 계산하기 위한 원본이다 —
        // 보정본(P*_cal)은 EKF가 잡은 영점이 이미 빠져 있어 역산이 되지 않는다.
        pressure_raw_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sensor/pressure_raw", 10, std::bind(&DataLoggerNode::pressure_raw_callback, this, std::placeholders::_1));

        rc_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/rc/command", 10, std::bind(&DataLoggerNode::rc_cmd_callback, this, std::placeholders::_1));

        rc_status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rc/status", 10, std::bind(&DataLoggerNode::rc_status_callback, this, std::placeholders::_1));

        motor_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor/output", 10, std::bind(&DataLoggerNode::motor_callback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor/tail_rpm", 10, std::bind(&DataLoggerNode::rpm_callback, this, std::placeholders::_1));

        // ---- 원시값 3종 (2026-08-18 추가) — 사후 진단용 상시 기록 ----
        imu_raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/raw/imu_6dof", 10, std::bind(&DataLoggerNode::imu_raw_callback, this, std::placeholders::_1));

        mag_raw_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/raw/magnetometer", 10, std::bind(&DataLoggerNode::mag_raw_callback, this, std::placeholders::_1));

        ekf_status_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/filtered/ekf_status", 10, std::bind(&DataLoggerNode::ekf_status_callback, this, std::placeholders::_1));

        hydro_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/filtered/hydro", 10, std::bind(&DataLoggerNode::hydro_callback, this, std::placeholders::_1));

        // 회차와 진입 때 잠근 설정 (auto_scenario_node, 100 Hz) — CSV 65~68열.
        run_info_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/auto/run_info", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
                if (m->data.size() < 4) return;
                run_id_    = static_cast<int32_t>(m->data[0]);
                thr_pct_   = m->data[1];
                angle_deg_ = m->data[2];
                run_sec_   = m->data[3];
            });

        // PID 진단 (pid_control_node, 100 Hz) — CSV 69~72열 + 74~76열. 게인·개루프 스위치는
        // 실행 중 바뀌므로 "그 순간 무엇으로 제어했나"가 같은 행에 남아야 사후 대조가 된다.
        // 2026-10-31: 발행이 4칸 -> 8칸이 되어 각속도 종속 제어 4칸이 붙었다. 크기 검사를
        // 4 와 8 로 나눈 이유는 **구 발행자(4칸)와도 계속 맞물려야** 하기 때문이다 —
        // pid_control_node 만 되돌리거나 먼저 띄워도 69~72열은 그대로 남고 74~77열만 NaN 이 된다.
        pid_debug_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pid/debug", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
                if (m->data.size() < 4) return;
                kp_pitch_     = m->data[0];
                pitch_ol_     = static_cast<int32_t>(m->data[1]);
                target_pitch_ = m->data[2];
                target_roll_  = m->data[3];
                if (m->data.size() < 8) return;
                rate_used_    = static_cast<int32_t>(m->data[4]);
                rate_meas_    = m->data[5];
                rate_err_     = m->data[6];
                kp_rate_      = m->data[7];
            });

        // AUTO 대기/주행 상태 (auto_scenario_node, 100 Hz). hydro 배열로 받으면 11 Hz 라
        // 전이 시각이 90 ms 뭉개지므로 직접 받는다. 0 IDLE / 1 WAIT / 2 RUN.
        // /auto/state 는 100*시나리오ID + 상태 다 (2026-10-14). 두 열로 나눠 적는다.
        // 로그만 보고 "무엇이 돌았는지"를 가릴 수 있어야 한다 — 10-03 로그에서 주행
        // 10회가 분명한데 어느 시나리오였는지 CSV 로 못 가려 저널을 뒤져야 했다.
        auto_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/auto/state", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                auto_state_  = msg->data % 100;
                scenario_id_ = msg->data / 100;
            });

        // 2번째 IMU (MPU9250, i2c_driver_node, 100 Hz) — 감시용. 자이로 바이어스가 튀는
        // 순간을 nRF IMU 와의 차이로 사후에 짚기 위한 것. i2c_driver_node 가 nRF IMU 칩 축으로
        // 돌려서 보내므로 24~29열과 열끼리 바로 뺄 수 있다.
        mpu_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/raw/imu_mpu9250", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() < 7) return;
                note_first_arrival(received_mpu_, "2번째 IMU(mpu9250)");
                for (int i = 0; i < 7; i++) mpu_[i] = msg->data[i];
            });

        // 100Hz 기록 타이머 (모든 토픽의 최신 스냅샷을 한 줄로)
        logging_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DataLoggerNode::write_log_loop, this));
    }

    ~DataLoggerNode() {
        // 소멸 시 버퍼에 남은 마지막 몇 줄까지 디스크에 확실히 내려쓴다.
        if (fp_ != nullptr) {
            fflush(fp_);
            fclose(fp_);
            RCLCPP_INFO(this->get_logger(), "[Data Logger] 로그 파일 보관 및 스트림 플러시 완료.");
        }
    }

private:
    int init_log_file() {
        const char* log_dir_path = "/home/fish/ros2_ws/log_csv";
        struct stat st;
        memset(&st, 0, sizeof(struct stat));

        // 디렉터리가 없으면 생성
        if (stat(log_dir_path, &st) == -1) {
            if (mkdir(log_dir_path, 0775) < 0) return -1;
        }

        // 파일명에 기동 시각을 박아 실행마다 새 파일이 생기게 한다 (덮어쓰기 사고 방지)
        time_t t = time(nullptr);
        struct tm tm_info;
        localtime_r(&t, &tm_info);   // _r 버전: 스레드 안전

        char file_path[512];
        snprintf(file_path, sizeof(file_path), "%s/fish_log_%04d%02d%02d_%02d%02d%02d.csv",
                 log_dir_path, tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);

        fp_ = fopen(file_path, "w");
        if (fp_ == nullptr) return -1;

        // 새 컬럼은 반드시 기존 컬럼 "뒤에" 덧붙인다 — 열 번호로 읽는 기존
        // 분석 스크립트(예전 C 펌웨어 규격 포함)가 깨지지 않게 하는 규칙이다.
        // 현재 36컬럼까지가 고정 구간이고, 37열부터가 이번 확장분이다.
        // AX..AZ[g], GX..GZ[도/초] — nRF 원시 규약 그대로 (REP-103 아님)
        // MX..MZ[µT] — 보정 전 칩 축. 원시로 남겨야 나중에 새 보정 계수를
        //              오프라인으로 다시 적용할 수 있다 (보정 후 값은 역산 불가)
        // BiasX..Z[도/초] — EKF 온라인 추정. EKF_Flags bit5(0x20)=지자기 두절(6축 폴백)
        // ※ 축 주의: AX..GZ·MX..MZ는 "칩 축", BiasX..Z는 "FLU 몸체 축"이다.
        //    원시 자이로에서 바이어스를 직접 빼면 틀린다 — 칩→몸체 변환
        //    (bx,by,bz) = (-gz, -gx, +gy) 를 거친 뒤 빼야 한다 (실측 대조 완료:
        //    BiasZ -3.06 이 원시 GY -3.0 에 대응).
        // P0_raw..P2_raw[mbar] — 영점 전 절대압. 보정본(P0_cal..)과 나란히 있어야
        //    (원시 - 보정) = EKF가 잡은 대기압 영점이 사후에 그대로 복원된다.
        //    수심·피토 차압을 나중에 다른 상수로 다시 계산하려면 이 원본이 필요하다.
        //    죽은 채널은 nan으로 찍힌다 (0.0 아님).
        // 40~47열: hydro. Hydro_Pitch 는 5~7열 Roll/Pitch 로 대체할 수 없다 —
        //   ① 오프라인 복원에 pressure_lag_ms 가 필요한데 그게 바로 구하려는 미지수다(순환)
        //   ② 없으면 Q_dyn = Q_raw - P_hydro - q_offset 을 감사할 수 없다
        //   ③ pitch 는 두 토픽에서 같은 값이라, 6열과의 상호상관 지연이 곧 파이프라인
        //      지연이다 — 열 하나로 공짜 지연 측정기를 얻는다
        // 48~55열: 대기창 물속 영점 (2026-09-16). AUTO 진입 대기 중 정지 구간의 q 를 그 주행의
        //   영점으로 잡은 것. 기존 Q_dyn/Speed 는 손대지 않고(비교용) 새 열로 나란히 낸다.
        //   Q_zero: 적용 중인 영점[mbar] (확정 전 nan, 없음이면 0) / Zero_src: 0 없음·1 정지 구간·2 폴백
        //   Zero_t0·t1: 영점 구간 시각 (Time(s) 축) / Still: WAIT 중 정지 판정 0/1
        //   Q_dyn_wz = Q_raw − P_hydro − Q_zero (필터 없음) / Speed_wz / State: 0 IDLE·1 WAIT·2 RUN
        // 65~73열: 회차·잠긴 설정·PID 상태·배터리 (2026-10-15). 로그 한 줄만 보고 "몇 번째
        //   주행이 어떤 설정·게인으로 돌았나"를 알 수 있게 한 것 — 지금까지는 저널을 뒤져야 했다.
        //   Run_id 는 AUTO 진입마다 +1 (노드 기동 시 0). Throttle_pct/Run_sec 는 **진입 때 잠근**
        //   값이고 Angle_deg 는 현재 파라미터다(단계표에 이미 녹아 있어 따로 잠그지 않는다).
        //   Kp_pitch/Pitch_OL 은 ros2 param 으로 주행 중에도 바뀌므로 매 행 기록한다.
        //   Target_Pitch/Target_Roll 은 AUTO 면 시나리오 목표, MANUAL 이면 스틱 목표다.
        //   VBat 은 23열 RSSI 와 같은 /rc/status 에서 오고, 센티볼트를 V 로 환산해 둔 것이다.
        //   ※ 페일세이프·static 구간에서는 /pid/debug 가 발행되지 않아 69~72열·74~77열에
        //     직전 값이 남는다. "제어 중인가"는 이 열들로 판정하지 말고 M1(12열)≠1500 이나
        //     State(55열)=2 로 가릴 것.
        // 74~77열: pitch 각속도 종속 제어 (2026-10-31). Kp_rate 가 0 이면 **각도 제어와 수학적으로
        //   동일**하고 Rate_err 는 출력에 들어가지 않는다(계산만 되어 기록된다 — 켜기 전에 "켰다면
        //   얼마나 개입했을지"를 미리 볼 수 있다). Kp_rate 는 물에서 ros2 param set 으로 0 -> 0.1 ->
        //   0.15 로 올려가며 튜닝하는 값이므로, Kp_pitch 를 69열에 넣은 것과 **같은 이유로** 매 행
        //   남긴다 — 어느 구간이 어느 게인이었는지 로그만으로 갈려야 한다.
        //   Rate_used: 그 주기에 각속도 항이 실제로 쓰였나 0/1 (Kp_rate=0 이거나 각속도가
        //     pitch_rate_max_age 보다 낡으면 0 — 즉 각도 제어로 복귀한 주기를 센다)
        //   Rate_meas: LPF 통과 pitch 각속도 [도/초] / Rate_err: 목표−측정 각속도 [도/초]
        // 64열: Scenario — AUTO 에 걸린 시나리오 ID (2026-10-14). 0 모름/MANUAL · 1 straight ·
        //   2 dive · 3 climb · 4 porpoise · 5 attitude · 6 bank_left · 7 bank_right · 8 static.
        //   **번호는 auto_scenario_node 의 SCEN_* enum 과 짝이다** — 재배정하면 지난 로그가 깨진다.
        //   MANUAL(AutoMode=0)·주행종료 구간에는 "다음에 돌 시나리오"가 들어간다. 실제로 돈
        //   구간을 보려면 State(55열)=2 로 먼저 거를 것.
        // 63열: Epoch(s) — UTC 유닉스 시각[초, ms 정밀도]. 로봇 밖 기록과 같은 시간선에 올리는 열.
        //   맨 뒤에 붙여 기존 열 번호를 하나도 건드리지 않았다 (머리말 "절대 시각" 참조).
        // 56~62열: MPU9250 6축 + 온도 (2026-09-16, 감시용 2번째 IMU). 단위는 24~29열과 같은 g·도/초,
        //   축도 **nRF IMU 칩 축으로 회전되어 온다** (i2c_driver_node, 2026-09-16 중력 실측 순열). 두 자이로 차 = 바이어스 차.
        fprintf(fp_, "Time(s),Btn1_Count,Mode,AutoMode,Roll,Pitch,Yaw,RC_T,RC_R,RC_P,RC_Y,M1,M2,M3,M4,RPM,P0_cal,P1_cal,P2_cal,T0,T1,T2,RSSI,"
                     "AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,BiasX,BiasY,BiasZ,EKF_Flags,"
                     "P0_raw,P1_raw,P2_raw,"
                     "Depth,Speed,Q_dyn,Q_raw,P_hydro,Hydro_Pitch,Hydro_Seq,Hydro_Flags,"
                     "Q_zero,Zero_src,Zero_t0,Zero_t1,Still,Q_dyn_wz,Speed_wz,State,"
                     "MPU_AX,MPU_AY,MPU_AZ,MPU_GX,MPU_GY,MPU_GZ,MPU_T,Epoch(s),Scenario,"
                     "Run_id,Throttle_pct,Angle_deg,Run_sec,Kp_pitch,Pitch_OL,Target_Pitch,Target_Roll,VBat,"
                     "Kp_rate,Rate_used,Rate_meas,Rate_err\n");
        fflush(fp_);   // 헤더는 즉시 기록해 파일이 비어 보이지 않게 한다

        return 0;
    }

    // 각 센서가 처음 붙는 시점을 로그 시간축 기준으로 남긴다. 로깅이 기동 즉시
    // 시작되므로, 초기값 구간과 실제 데이터 구간의 경계를 이걸로 찾을 수 있다.
    void note_first_arrival(bool &flag, const char *what) {
        if (flag) return;
        flag = true;
        double t = is_logging_active_ ? (this->now() - start_time_).seconds() : 0.0;
        RCLCPP_INFO(this->get_logger(), "[Data Logger] %s 수신 시작 (t = %.3f초).", what, t);
    }

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        note_first_arrival(received_attitude_, "자세(attitude)");
        // state_estimation이 자동 모드 표시용으로 roll에 실은 ±5000 오프셋을 제거해
        // 실제 자세각을 기록한다 (pid_control_node의 디코딩과 동일 규칙).
        // 동시에 이 오프셋을 로봇의 실제 모드(단일 출처)로 사용한다.
        if (msg->x > 2500.0f) {
            roll_ = msg->x - 5000.0f;
            is_auto_mode_ = true;
        } else if (msg->x < -2500.0f) {
            roll_ = msg->x + 5000.0f;
            is_auto_mode_ = true;
        } else {
            roll_ = msg->x;
            is_auto_mode_ = false;
        }
        pitch_ = msg->y;
        yaw_   = msg->z;
    }

    void pressure_cal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) return;
        note_first_arrival(received_pressure_, "보정 압력(pressure_calibrated)");
        p0_cal_ = msg->data[0];
        p1_cal_ = msg->data[1];
        p2_cal_ = msg->data[2];
    }

    // 원시 압력 토픽: [0..2] 절대압(mbar, 영점 전), [3..5] 채널별 온도(C).
    // 죽은 채널은 드라이버가 NaN으로 채워 보내므로 그대로 통과시킨다 — 여기서
    // 거르면 CSV에서 "고장"과 "정상적으로 0"을 구별할 수 없게 된다.
    void pressure_raw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        p0_raw_ = msg->data[0];
        p1_raw_ = msg->data[1];
        p2_raw_ = msg->data[2];
        t0_ = msg->data[3];
        t1_ = msg->data[4];
        t2_ = msg->data[5];
    }

    // 조종기 스틱 값. -9999(두절 센티넬)도 걸러내지 않고 그대로 기록해
    // 나중에 로그에서 두절 구간을 눈으로 찾을 수 있게 한다.
    void rc_cmd_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        rc_r_ = static_cast<int16_t>(msg->x);
        rc_p_ = static_cast<int16_t>(msg->y);
        rc_y_ = static_cast<int16_t>(msg->z);
        rc_t_ = static_cast<int16_t>(msg->w);
    }

    void rpm_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        rpm_ = msg->data;
    }

    void rc_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 5) return;
        int current_btn1 = msg->data[0];
        rssi_ = msg->data[4];
        // 배터리 전압 — 지금까지 받고도 버리던 값이다(73열 VBat, 2026-10-15).
        // **센티볼트다** — 100 으로 나눠야 V 다 (CLAUDE.md §3). 기록 시점에 나눈다.
        vbat1_ = msg->data[2];

        // 짧은 누름(Edge Detection)으로 버튼 누적 카운트만 집계한다.
        // 자동/수동 모드 판정은 여기서 하지 않고 state_estimation이 방송한
        // /filtered/attitude 오프셋(attitude_callback)을 단일 출처로 사용한다.
        //
        // /rc/status는 100Hz로 오므로 카운터 1틱 = 10ms다.
        if (current_btn1 == 1) {
            btn1_hold_counter_++;
            if (btn1_hold_counter_ == 300) {   // 3초 이상 = 롱프레스 -> 짧은 누름 집계에서 제외
                btn1_long_processed_ = true;
            }
        } else {
            // 진짜 뗌(current==0)만 짧은 누름으로 인정한다. 통신두절 시 btn=255로 튀는
            // 1→255 전이를 '뗌'으로 오인해 카운트가 증가하는 것을 막는다(N4).
            if (prev_btn1_ == 1 && current_btn1 == 0) {
                // 손을 떼는 순간(Falling Edge), 30ms 이상 1초 이하로 눌렸을 때만 짧은 누름으로 인정
                // (하한: 채터링 제거 / 상한: 롱프레스와 구분)
                if (btn1_hold_counter_ > 3 && btn1_hold_counter_ < 100 && !btn1_long_processed_) {
                    btn1_cumulative_count_++; // 누적 카운트 증가!
                }
            }
            btn1_hold_counter_ = 0;
            btn1_long_processed_ = false;
        }
        prev_btn1_ = current_btn1;

        // 통신 상태만 결정한다 (0 = 정상, 2 = 조종기 두절 -9999 센티넬).
        // 자동/수동 여부는 별도 AutoMode 컬럼(is_auto_mode_)으로 분리 기록한다.
        if (rc_t_ == -9999) {
            mode_ = 2;
        } else {
            mode_ = 0;
        }
    }

    void motor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        m1_ = msg->data[0];
        m2_ = msg->data[1];
        m3_ = msg->data[2];
        m4_ = msg->data[3];
    }

    // 원시 6축. nRF가 보낸 g·도/초 단위 그대로다(REP-103 아님) — 변환 없이
    // 기록해야 발행 당시 값과 1:1로 대조된다.
    void imu_raw_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ax_ = static_cast<float>(msg->linear_acceleration.x);
        ay_ = static_cast<float>(msg->linear_acceleration.y);
        az_ = static_cast<float>(msg->linear_acceleration.z);
        gx_ = static_cast<float>(msg->angular_velocity.x);
        gy_ = static_cast<float>(msg->angular_velocity.y);
        gz_ = static_cast<float>(msg->angular_velocity.z);
    }

    void mag_raw_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        mx_ = static_cast<float>(msg->x);
        my_ = static_cast<float>(msg->y);
        mz_ = static_cast<float>(msg->z);
    }

    // EKF 진단(10Hz 발행이라 CSV에는 같은 값이 ~10행씩 반복된다 — 정상).
    // 바이어스는 세션 간 50배씩 다르고 열로도 흐르므로(2시간에 0.06도/초 실측)
    // 자세 이상 시 "바이어스 폭주였나"를 이 열이 즉답한다.
    void ekf_status_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 12) return;
        bias_x_ = msg->data[0];
        bias_y_ = msg->data[1];
        bias_z_ = msg->data[2];
        ekf_flags_ = static_cast<int32_t>(msg->data[10]);
    }

    // /filtered/hydro = [depth, speed, q, q_raw, dP_hydro, static, pitch_used, flags,
    //                    wz_zero, wz_src, wz_t0_age, wz_t1_age, wz_still, q_wz, speed_wz]  (8~14: 2026-09-16)
    void hydro_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 8) return;
        note_first_arrival(received_hydro_, "수심/속도(hydro)");
        depth_       = msg->data[0];
        speed_       = msg->data[1];
        q_dyn_       = msg->data[2];
        q_raw_       = msg->data[3];
        p_hydro_     = msg->data[4];
        hydro_pitch_ = msg->data[6];
        hydro_flags_ = static_cast<int32_t>(msg->data[7]);
        hydro_seq_++;   // 11Hz 신호가 100Hz CSV에서 같은 값 9행으로 늘어나므로,
                        // 새 샘플이 어느 행에 떨어졌는지 이것 없이는 식별 불가다

        // 대기창 영점 (2026-09-16). 옛 8칸 발행자면 nan/0 으로 남긴다.
        const float nf = std::numeric_limits<float>::quiet_NaN();
        if (msg->data.size() >= 15) {
            q_zero_   = msg->data[8];
            zero_src_ = static_cast<int32_t>(msg->data[9]);
            still_    = static_cast<int32_t>(msg->data[12]);
            q_dyn_wz_ = msg->data[13];
            speed_wz_ = msg->data[14];
            // 구간 시각은 "지금보다 몇 초 전"(나이)으로 온다 — float32 에 epoch 초를 실으면
            // 128초 단위로 뭉개지기 때문이다. 여기서 Time(s) 축으로 환산한다.
            const double t_now = is_logging_active_ ? (this->now() - start_time_).seconds() : NAN;
            zero_t0_ = (std::isfinite(msg->data[10]) && std::isfinite(t_now)) ? (float)(t_now - msg->data[10]) : nf;
            zero_t1_ = (std::isfinite(msg->data[11]) && std::isfinite(t_now)) ? (float)(t_now - msg->data[11]) : nf;
        } else {
            q_zero_ = nf; zero_src_ = 0; still_ = 0; q_dyn_wz_ = nf; speed_wz_ = nf;
            zero_t0_ = nf; zero_t1_ = nf;
        }
    }

    void write_log_loop() {
        // 센서 연결 여부와 무관하게 노드 기동 즉시 로깅을 시작한다.
        // (이전에는 received_attitude_ && received_pressure_ 를 요구했는데, 압력센서를
        //  물리적으로 떼고 벤치 테스트하면 /sensor/pressure_calibrated가 영원히 발행되지
        //  않아 CSV에 헤더만 남고 데이터가 0행이었다. 각 센서가 붙기 전 구간은 초기값으로
        //  기록되며, 실제 수신 시작 시점은 아래 콜백들이 INFO 로그로 남긴다.)
        if (!is_logging_active_) {
            is_logging_active_ = true;
            start_time_ = this->now();   // 이 시각이 Time(s) 컬럼의 0초
            RCLCPP_INFO(this->get_logger(), "=========================================");
            {
                // 다른 장비 기록과 맞출 때 쓸 기준점을 저널에 남긴다. CSV 의 Epoch(s) 와
                // 같은 시계이고, Time(s)=0 이 이 시각에 해당한다.
                struct timespec ts0;
                clock_gettime(CLOCK_REALTIME, &ts0);
                struct tm tm0;
                localtime_r(&ts0.tv_sec, &tm0);
                char buf[64];
                strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm0);
                RCLCPP_INFO(this->get_logger(),
                    "[Data Logger] 데이터 로깅을 0.0초부터 시작합니다. (Time(s)=0 => %s.%03ld, Epoch %.3f)",
                    buf, ts0.tv_nsec / 1000000, (double)ts0.tv_sec + ts0.tv_nsec * 1e-9);
            }
            RCLCPP_INFO(this->get_logger(), "[Data Logger] (센서 미연결 구간은 초기값으로 기록됩니다)");
            RCLCPP_INFO(this->get_logger(), "=========================================");
        }

        if (fp_ == nullptr) return;

        double relative_time_sec = (this->now() - start_time_).seconds();

        // 절대 시각 — **벽시계를 직접 읽는다.** this->now() 는 노드 시계(기본 시스템 시계와
        // 같지만 use_sim_time 등으로 갈릴 수 있다)이므로, 다른 장비와 맞추는 값은 OS 시계에서
        // 바로 가져와 해석이 한 가지로 고정되게 한다.
        struct timespec ts_now;
        clock_gettime(CLOCK_REALTIME, &ts_now);
        const double epoch_sec = (double)ts_now.tv_sec + (double)ts_now.tv_nsec * 1e-9;

        // btn1_cumulative_count_ 를 기록합니다.
        // 자릿수: 가속도/바이어스 %.4f (바이어스 불안정성 실측이 0.001도/초 수준),
        //         자이로 %.3f, 지자기 %.2f (노이즈 ~0.1µT 아래는 의미 없음)
        //         원시 압력 %.3f — 0.001mbar = 수심 0.01mm. 실측 잡음이 0.13mbar라
        //         기존 %.2f로도 버티지만, 사후 재계산의 원본이라 잘라 둘 이유가 없다.
        fprintf(fp_, "%.3f,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%u,%u,%u,%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,"
                     "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%d,"
                     "%.3f,%.3f,%.3f,"
                     "%.4f,%.3f,%.4f,%.4f,%.4f,%.2f,%d,%d,"
                     "%.4f,%d,%.2f,%.2f,%d,%.4f,%.3f,%d,"
                     "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.1f,"
                     "%.3f,%d,"
                     "%d,%.1f,%.1f,%.2f,%.3f,%d,%.2f,%.2f,%.2f,"
                     "%.3f,%d,%.3f,%.3f\n",
                relative_time_sec,
                btn1_cumulative_count_,
                mode_,
                is_auto_mode_ ? 1 : 0,
                roll_, pitch_, yaw_,
                rc_t_, rc_r_, rc_p_, rc_y_,
                m1_, m2_, m3_, m4_,
                rpm_,
                p0_cal_, p1_cal_, p2_cal_,
                t0_, t1_, t2_,
                rssi_,
                ax_, ay_, az_,
                gx_, gy_, gz_,
                mx_, my_, mz_,
                bias_x_, bias_y_, bias_z_,
                ekf_flags_,
                p0_raw_, p1_raw_, p2_raw_,
                depth_, speed_, q_dyn_, q_raw_, p_hydro_, hydro_pitch_, hydro_seq_, hydro_flags_,
                q_zero_, zero_src_, zero_t0_, zero_t1_, still_, q_dyn_wz_, speed_wz_, auto_state_,
                mpu_[0], mpu_[1], mpu_[2], mpu_[3], mpu_[4], mpu_[5], mpu_[6],
                epoch_sec, scenario_id_,
                run_id_, thr_pct_, angle_deg_, run_sec_,
                kp_pitch_, pitch_ol_, target_pitch_, target_roll_, vbat1_ / 100.0f,
                kp_rate_, rate_used_, rate_meas_, rate_err_);

        // 50줄(0.5초)마다 디스크로 강제 플러시.
        // 매 줄 플러시하면 SD카드 I/O로 100Hz 루프가 밀리고, 아예 안 하면
        // 전원이 갑자기 끊길 때 마지막 수 초가 통째로 날아간다. 그 절충점.
        static int flush_cnt = 0;
        if (++flush_cnt >= 50) {
            fflush(fp_);
            flush_cnt = 0;

            // 파일당 200MB 회전. 판정은 플러시 직후에만 하므로 ftell이 곧 실제
            // 파일 크기다. start_time_은 건드리지 않는다 — Time(s)가 파일 경계를
            // 넘어 이어져야 회전된 파일들을 그대로 이어붙여 분석할 수 있다.
            if (ftell(fp_) >= ROTATE_BYTES) {
                fclose(fp_);
                fp_ = nullptr;
                if (init_log_file() < 0) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "[Data Logger] 로그 회전 실패 — 기록 중단 (디스크 용량 확인 필요)");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),
                            "[Data Logger] 200MB 도달 — 새 파일로 회전 (t = %.1f초, Time축 연속).",
                            relative_time_sec);
            }
        }
    }

    FILE* fp_;

    bool received_attitude_;    // 최초 수신 로그를 한 번만 찍기 위한 래치
    bool received_pressure_;
    bool received_hydro_ = false;
    bool received_mpu_   = false;
    bool is_logging_active_;
    rclcpp::Time start_time_;   // Time(s) 컬럼의 기준 시각

    // [추가] 버튼 엣지 디텍션 및 상태 관리용 변수들
    int32_t btn1_cumulative_count_;   // 짧은 누름 누적 횟수 (CSV Btn1_Count 컬럼)
    int btn1_hold_counter_;           // 현재 눌림 지속 틱 수 (1틱 = 10ms)
    int prev_btn1_;                   // 직전 버튼 상태 (엣지 판정용)
    bool btn1_long_processed_;        // 롱프레스로 이미 처리됨 -> 뗄 때 짧은 누름 집계 안 함
    bool is_auto_mode_;

    // ---- 이하가 CSV 한 줄을 구성하는 최신 스냅샷 ----
    int32_t mode_;                             // 0=정상, 2=조종기 두절
    float roll_, pitch_, yaw_;
    int16_t rc_t_, rc_r_, rc_p_, rc_y_;
    uint16_t m1_, m2_, m3_, m4_;
    int32_t rpm_;
    float p0_cal_, p1_cal_, p2_cal_;
    float t0_, t1_, t2_;      // 채널별 원시 온도(C) — 압력 드리프트 진단용
    float p0_raw_, p1_raw_, p2_raw_;  // 채널별 원시 절대압(mbar, 영점 전). 고장/미수신 = NaN
    float depth_, speed_, q_dyn_, q_raw_, p_hydro_, hydro_pitch_;   // hydro. 미수신 = NaN
    int32_t hydro_seq_, hydro_flags_;
    float q_zero_, zero_t0_, zero_t1_, q_dyn_wz_, speed_wz_;      // 대기창 영점. 미확정 = NaN
    int32_t zero_src_, still_, auto_state_;                       // 0 없음/1 정지/2 폴백, 0/1, 0 IDLE/1 WAIT/2 RUN
    int32_t scenario_id_;                                         // 시나리오 ID (auto_scenario_node SCEN_* 와 짝)
    int32_t run_id_;                                              // AUTO 진입 회차 (65열)
    float   thr_pct_, angle_deg_, run_sec_;                       // 진입 때 잠근 설정 (66~68열)
    float   kp_pitch_;                                            // 그 순간 pitch 게인 (69열)
    int32_t pitch_ol_;                                            // pitch 개루프 0/1 (70열)
    float   target_pitch_, target_roll_;                          // 그 순간 목표 각도 [도] (71~72열)
    float   kp_rate_;                                             // 각속도 종속 게인 [초] (74열, 0=꺼짐)
    int32_t rate_used_;                                           // 각속도 항 실사용 0/1 (75열)
    float   rate_meas_, rate_err_;                                // 측정·오차 각속도 [도/초] (76~77열)

    float mpu_[7];                                                // MPU9250 [ax ay az g, gx gy gz 도/초, 온도 °C]. 미수신 = NaN
    int32_t rssi_;
    int32_t vbat1_;                                               // 배터리 전압 [센티볼트] (73열, ÷100 해서 기록)
    float ax_, ay_, az_, gx_, gy_, gz_;      // 원시 6축 [g, 도/초]
    float mx_, my_, mz_;                     // 원시 지자기 [µT], 보정 전 칩 축
    float bias_x_, bias_y_, bias_z_;         // EKF 자이로 바이어스 추정 [도/초]
    int32_t ekf_flags_;                      // EKF 상태 비트 (bit5 = 6축 폴백)

    static constexpr long ROTATE_BYTES = 200L * 1024 * 1024;   // 파일당 회전 문턱

    rclcpp::TimerBase::SharedPtr logging_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_cal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pressure_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr rc_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rc_status_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mag_raw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ekf_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hydro_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr auto_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr run_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pid_debug_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpu_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataLoggerNode>();
    // 생성자에서 파일 열기에 실패하면 rclcpp::shutdown()이 호출된 상태일 수 있으므로
    // spin 전에 한 번 확인한다.
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
