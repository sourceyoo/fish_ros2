// =============================================================================
//  auto_scenario_node.cpp
//
//  역할 : 자율 주행(AUTO) 모드일 때 시간 기반으로 목표 자세를 생성하는 궤적 엔진
//
//   [입력] /filtered/attitude (Vector3)  <- 자세 + 모드 플래그(±5000 오프셋)
//   [출력] /auto/command     (Quaternion) -> pid_control_node의 목표값
//                                         z = 서보 고정 플래그 (0 평상시 / 1 = 완전 정지, 2026-09-16)
//          /auto/run_info    (Float32MultiArray[4]) -> [회차, 추력%, 각도도, 주행초] — 진입 때 잠근 값
//                                           (2026-10-15). data_logger_node 가 CSV 65~68열로 쓴다.
//                                           회차는 **AUTO 진입마다 1 증가**하는 번호다(노드 기동 시 0).
//                                           사후 분석에서 "몇 번째 주행의 설정이 무엇이었나"를
//                                           로그 한 줄에서 바로 읽기 위한 것 — 지금까지는 저널을
//                                           뒤져야 했다. 매 틱 발행하므로 MANUAL 구간에는 **직전
//                                           주행의 값**이 남는다(아직 한 번도 안 달렸으면 회차 0).
//          /auto/state       (Int32)      -> **100*시나리오ID + 상태** 로 둘을 한 숫자에 싣는다 (2026-10-14)
//                                           상태  = 값 % 100 : 0 IDLE / 1 WAIT(진입 대기) / 2 RUN(주행)
//                                           시나리오ID = 값 / 100 : 아래 SCENARIO_ID 표
//                                           hydro_estimator_node 는 상태만 보고(대기창 영점 시점),
//                                           data_logger_node 는 둘로 나눠 CSV State·Scenario 열로 쓴다.
//
//  ── 왜 한 숫자에 싣나 (2026-10-14) ────────────────────────────────────────
//  "로그만 보고 무엇이 돌았는지 알 수 없다"는 문제가 실제로 걸렸다 — 10-03 로그는
//  주행 10회가 분명한데 straight 인지 dive 인지 bank 인지 CSV 로는 못 가렸고 저널을
//  뒤져야 했다. 그래서 시나리오를 CSV 에 남긴다.
//  문자열 토픽을 새로 만들지 않고 기존 /auto/state 에 끼운 이유:
//    · CSV 가 숫자 열이라 awk/python 으로 거르기 쉽다 (문자열은 쉼표·따옴표 문제가 붙는다)
//    · 상태와 시나리오는 **같은 순간의 한 사실**이다. 토픽이 둘이면 전이 틱에서 서로
//      어긋난 조합(예: RUN 인데 직전 시나리오)이 로그에 남을 수 있다
//    · 구독자(hydro)가 `% 100` 한 줄만 더하면 되고 기존 판정이 그대로 간다
//  ※ 새 ID 를 추가하면 **세 파일을 같이 고쳐야 한다** — 여기, data_logger_node 의
//    해독표, 그리고 docs/csv_format.md 의 표. ±5000 모드 규약과 같은 종류의 약속이다.
//
//  ── 시나리오 교체 방식 (2026-09-03) ────────────────────────────────────────
//  시나리오는 **단계표(Step 배열)** 하나로 표현한다. 단계마다 지속시간과
//  목표 자세·추력을 적어두면 되고, 새 시나리오를 넣는 일은 표를 하나 더
//  쓰는 것이 전부다(build_steps() 참조).
//
//  ── 조종기로 전환 (2026-09-03) ────────────────────────────────────────────
//  **btn1+btn2 동시 누름** = 시나리오 전환. state_estimation_ekf_node 가 조합을
//  감지해 `/ui/scenario_next` 를 발행하고, 이 노드가 `combo_list` 를 한 칸 돌린다.
//  **2026-10-14 현재 기본값은 scenario=dive, combo_list="dive,straight"** (운용자 요청) —
//  버튼을 하나도 안 눌러도 **AUTO 진입 첫 시나리오가 dive(잠수)** 이고, 조합 버튼으로
//  직진과 번갈아 바꾼다. 뱅크 선회는 코드에 남아 있지만 버튼 목록에서는 뺐다.
//  이 기본값 두 줄은 측정과 주행 사이를 여러 번 오갔다 — 10-01 static 고정, 10-03 주행 복귀,
//  10-05 재고정, 10-14 dive + 콤보 둘로. **시나리오 코드는 매번 그대로 뒀다.**
//  센서 측정이나 다른 조합이 필요하면 파라미터로 넣는다:
//     ros2 param set /auto_scenario_node scenario static
//     ros2 param set /auto_scenario_node combo_list "dive,straight,bank_left,bank_right"
//
//  ※ 조합은 **모드를 바꾸지 않는다.** 시나리오만 갈아 끼우고, 실제 주행은 btn1 단독
//     누름으로 AUTO 에 들어갈 때 시작된다. 전환은 저널에 크게 남는다 — 지금 무엇이
//     걸려 있는지는 로봇을 봐서는 알 수 없으므로 로그가 유일한 확인 수단이다.
//
//  선택은 ROS 파라미터로도 한다. 재빌드도 재시작도 필요 없다:
//     ros2 param set /auto_scenario_node scenario dive
//     ros2 param set /auto_scenario_node run_sec 5.0
//     ros2 param set /auto_scenario_node throttle_pct 30.0
//     ros2 param set /auto_scenario_node angle_deg 15.0
//     ros2 param set /auto_scenario_node start_delay_sec 10.0   # 진입 후 대기(초), 0.0 = 즉시
//
//  **바뀐 값은 다음 AUTO 진입부터 적용된다** (달리는 도중에 궤적이 갈아엎히면
//  물속에서 무슨 일이 일어난 건지 사후에 못 가린다). btn1 로 나갔다 들어오면 된다.
//
//  공통 규칙 세 가지:
//    · **AUTO 에 들어가면 start_delay_sec(기본 10초) 동안 기다렸다가 시작한다** (2026-09-15, 09-19 10초로).
//      기다리는 동안은 추력 0 으로 수평만 지시하고, run_sec 은 대기가 끝난 순간부터 센다.
//      대기 중 MANUAL 로 나가면 취소되고, 다시 들어오면 처음부터 기다린다.
//      대기 중임은 /auto/state = 1 로 알린다 — hydro_estimator_node 가 이 구간에서 물속
//      동압 영점을 모으고, 주행(= 2)으로 넘어가는 순간 확정한다 (2026-09-16). 버튼 직후
//      4~5초는 가라앉는 중(0915 실측 수심 +14~21 mm/s)이라 5초로는 정지 구간 2초가 안 잡혀
//      폴백으로 빠진다. 그래서 2026-09-19 부터 기본 10초다 — 가라앉는 시간 + 정지 2초 + 여유.
//    · 어떤 시나리오든 **run_sec 초가 지나면 추력을 끊고 수평으로 정지**한다.
//      단계표가 그보다 짧으면 표를 처음부터 되풀이한다(왕복 패턴이 이렇게 나온다).
//      **예외: 지속 시나리오(open_ended, 지금은 static).** 측정을 미리 몇 초라고 정해 두는 것이
//      말이 안 되는 시나리오라, run_sec 을 아예 보지 않고 btn1 으로 MANUAL 에 나올 때까지
//      계속 간다 (2026-10-01, 운용자 요청). 끝은 사람이 정한다.
//    · 단계의 추력 -1 은 "throttle_pct 를 쓴다"는 뜻이다. 운용자가 안전 추력을
//      한 곳에서만 낮출 수 있게 하기 위한 것이다.
//
//  ── 시나리오 목록 ─────────────────────────────────────────────────────────
//    straight  (기본) 수평 직진.                     ← 물속 첫 검증용
//    dive      코를 angle_deg 만큼 아래로 하고 전진   (이 로봇은 **양수 피치 = 코 아래**)
//    climb     코를 angle_deg 만큼 위로 하고 전진
//    porpoise  3초마다 피치 ±10° 왕복하며 전진        (구 시나리오. run_sec 을 늘려 쓸 것)
//    attitude  추력 0 으로 피치 ±10° 왕복             ← 벤치에서 서보만 볼 때
//    static    **완전 정지** — 서보 3개 1500 고정, 추력 0. 센서만 측정 (2026-09-16).
//              **run_sec 을 무시하고 MANUAL 로 나올 때까지 계속한다** (2026-10-01)
//    bank_left  수평 피치로 롤을 angle_deg 만큼 **왼쪽**으로 기울여 전진   (2026-09-16)
//    bank_right 같은 것을 **오른쪽**으로
//
//  ── 뱅크 선회 (2026-09-16) ────────────────────────────────────────────────
//  요 서보(data[2])는 계속 1500 고정이다. 회전은 **몸을 기울여** 만든다 — 물고기·비행기가
//  도는 방식이고, 좌우 지느러미가 롤을 만드는 기존 믹싱을 그대로 쓴다. 롤 부호는 2026-09-16
//  벤치에서 실측: **롤 − = 왼쪽으로 기움, 롤 + = 오른쪽으로 기움** (좌 −34.5, 우 +40.5).
//  기울면 어느 쪽으로 도는지·얼마나 도는지는 물속에서 봐야 한다 — 회전량은 CSV Yaw 열
//  (양수 = 좌선회, 반시계)로 잰다. 요는 제어에 쓰지 않고 측정에만 쓴다.
//
//  ※ 요 목표 없음 (2026-09-14, 운용자 결정): pid_control_node 가 요 제어를 걷어내
//     요 서보를 중앙에 고정했으므로 이 노드도 헤딩을 다루지 않는다. 헤딩을 틀던
//     `turn` 시나리오도 뺐다 (지정하면 "모르는 시나리오" 로 거부된다).
//     비어 버린 /auto/command 의 z 칸은 2026-09-16 부터 **서보 고정 플래그**로 쓴다
//     (아래 static 참조) — 요 목표가 아니다.
//
//  ── static: 완전 정지 (2026-09-16) ────────────────────────────────────────
//  "추력 0, 롤·피치 0" 만으로는 가만히 있지 않는다 — PID 가 **수평을 유지하려고**
//  지느러미를 계속 움직이기 때문이다. 로봇이 조금만 기울어도 오차에 비례해 서보가 돈다.
//  그래서 이 시나리오는 단계표의 hold 칸을 세워 /auto/command z = 1 을 보내고,
//  pid_control_node 가 그걸 보면 PID 를 통째로 건너뛰고 [1500,1500,1500,1000] 을 낸다.
//  쓰임: 구동 없이 압력·IMU 만 재는 기준 측정. 물속 수심 드리프트, 자이로 바이어스,
//  그리고 **대기창 물속 영점 검증** — 완전 정지이므로 Speed_wz 가 0 근처여야 맞다.
//
//  **길이는 사람이 정한다 (2026-10-01).** 처음에는 run_sec 을 썼는데, 측정을 몇 초 할지
//  미리 정해 두고 그 숫자를 파라미터로 바꿔가며 실험하는 게 실제 운용과 맞지 않았다
//  (2026-09-30 실기). 그래서 static 은 Step 의 open_ended 칸을 세워 **run_sec 을 보지 않고**
//  btn1 으로 MANUAL 에 나올 때까지 계속한다. 그동안 CSV 는 100Hz 로 계속 쌓이므로
//  "State(55열)가 2 인 구간" 이 곧 측정 구간이다.
//
//  ※ 모드 전달 규약: state_estimation_ekf_node가 자동 모드일 때 roll 값에 ±5000을
//     더해 발행한다. 별도 모드 토픽 없이 자세 토픽 하나로 모드를 함께 실어 나르는
//     방식이며, 이 노드와 pid_control_node / data_logger_node가 동일하게 디코딩한다.
//
//  ※ 주의: AUTO 에서는 pid_control_node 의 조종기 페일세이프가 통째로 건너뛰어진다
//     (조종기 없이 도는 것이 정상 사용이므로). AUTO 중에는 송신기를 꺼도 멈추지
//     않는다 — 멈추려면 btn1 을 눌러 MANUAL 로 나와야 한다.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

// /auto/state 값. hydro_estimator_node·data_logger_node 가 같은 숫자로 해독한다 —
// 바꾸면 세 파일을 같이 바꿔야 한다 (±5000 모드 규약과 같은 종류의 약속).
enum { AUTO_STATE_IDLE = 0, AUTO_STATE_WAIT = 1, AUTO_STATE_RUN = 2 };

// 시나리오 ID — /auto/state 에 100 배수로 실린다. **번호를 재배정하지 말 것**:
// 지난 로그의 Scenario 열이 이 숫자로 해석되므로, 새 시나리오는 뒤에 붙인다.
// 0 은 "모름"(MANUAL 이거나 이름을 못 읽은 경우)이다.
enum {
    SCEN_NONE = 0, SCEN_STRAIGHT = 1, SCEN_DIVE = 2, SCEN_CLIMB = 3,
    SCEN_PORPOISE = 4, SCEN_ATTITUDE = 5, SCEN_BANK_LEFT = 6,
    SCEN_BANK_RIGHT = 7, SCEN_STATIC = 8
};

// 이름 -> ID. is_known_scenario() 와 **같은 목록이어야 한다**(모르는 이름은 0).
static int scenario_id_of(const std::string &s) {
    if (s == "straight")   return SCEN_STRAIGHT;
    if (s == "dive")       return SCEN_DIVE;
    if (s == "climb")      return SCEN_CLIMB;
    if (s == "porpoise")   return SCEN_PORPOISE;
    if (s == "attitude")   return SCEN_ATTITUDE;
    if (s == "bank_left")  return SCEN_BANK_LEFT;
    if (s == "bank_right") return SCEN_BANK_RIGHT;
    if (s == "static")     return SCEN_STATIC;
    return SCEN_NONE;
}

class AutoScenarioNode : public rclcpp::Node {
public:
    // 시나리오 한 단계. 각도는 도(deg), 추력은 % (음수면 throttle_pct 를 쓴다).
    // 요 칸은 없다 — 요 제어 없음 (2026-09-14).
    // hold       = 서보 고정(완전 정지).
    // open_ended = run_sec 을 무시하고 MANUAL 로 나올 때까지 계속 (지속 시나리오).
    // 기존 표들은 두 칸을 안 적으므로 기본값 false 다.
    struct Step {
        double sec;
        double roll;
        double pitch;
        double thr_pct;
        bool   hold       = false;
        bool   open_ended = false;
    };

    AutoScenarioNode() : Node("auto_scenario_node"),
                         is_auto_mode_(false),
                         prev_auto_mode_(false),
                         run_done_logged_(false)
    {
        // ── 파라미터 (기본값의 진실원은 여기 한 곳) ────────────────────────
        // 2026-10-14: 운용자 요청으로 **AUTO 진입 첫 시나리오를 dive** 로 둔다. 버튼을 하나도
        // 안 눌러도 btn1 진입만으로 잠수가 돈다.
        // 이 기본값 한 줄은 측정과 주행 사이를 여러 번 오갔다 — 10-01 static, 10-03 straight,
        // 10-05 static, 10-14 dive. **시나리오 코드는 매번 그대로 뒀다.**
        // 센서 측정(완전 정지)으로 돌아갈 때:
        //   ros2 param set /auto_scenario_node scenario static
        // ※ combo_list 는 "dive,straight" 다 (10-14) — 조합 버튼으로 직진과 오간다.
        scenario_     = this->declare_parameter<std::string>("scenario", "dive");
        run_sec_      = this->declare_parameter<double>("run_sec", 3.0);
        throttle_pct_ = this->declare_parameter<double>("throttle_pct", 40.0);
        angle_deg_    = this->declare_parameter<double>("angle_deg", 15.0);
        start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 10.0);   // 5 -> 10 (2026-09-19, 대기창 영점 확보용)
        if (start_delay_sec_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "[AUTO] start_delay_sec %.1f 는 음수라 0 으로 둡니다", start_delay_sec_);
            start_delay_sec_ = 0.0;
        }
        // btn1+btn2 3초마다 **잠수 <-> 직진** 으로 한 칸 돈다 (2026-10-14, 운용자 요청).
        // 물가에서 노트북 없이 바꿀 수 있게 둘만 남겼다 — 뱅크 선회는 목록에서 뺐다.
        // ※ scenario 기본값이 dive 라 **버튼을 안 눌러도 AUTO 진입 = dive** 다. 콤보는
        //   straight 로 옮기고 다시 누르면 dive 로 돌아온다 (목록이 둘뿐이라 번갈아).
        // 뱅크까지 쓰려면: ros2 param set /auto_scenario_node combo_list "dive,straight,bank_left,bank_right"
        combo_list_   = parse_list(this->declare_parameter<std::string>("combo_list", "dive,straight"));

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &ps) {
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                for (const auto &p : ps) {
                    const auto &n = p.get_name();
                    if (n == "scenario") {
                        const std::string v = p.as_string();
                        if (!is_known_scenario(v)) {
                            r.successful = false;
                            r.reason = "모르는 시나리오: " + v +
                                       " (straight/dive/climb/porpoise/attitude/bank_left/bank_right/static"
                                       " — turn 은 요 제어 제거로 없음)";
                            RCLCPP_ERROR(this->get_logger(), "[AUTO] %s", r.reason.c_str());
                            continue;
                        }
                        scenario_ = v;
                    } else if (n == "combo_list") {
                        auto v = parse_list(p.as_string());
                        if (v.empty()) {
                            r.successful = false;
                            r.reason = "combo_list 가 비었거나 모르는 이름만 있습니다";
                            RCLCPP_ERROR(this->get_logger(), "[AUTO] %s", r.reason.c_str());
                            continue;
                        }
                        combo_list_ = v; combo_idx_ = 0;
                    } else if (n == "run_sec")      run_sec_      = p.as_double();
                    else if (n == "throttle_pct")   throttle_pct_ = p.as_double();
                    else if (n == "angle_deg")      angle_deg_    = p.as_double();
                    else if (n == "start_delay_sec") {
                        const double v = p.as_double();
                        if (v < 0.0) {
                            r.successful = false;
                            r.reason = "start_delay_sec 는 0 이상이어야 합니다 (0.0 = 진입 즉시 시작)";
                            RCLCPP_ERROR(this->get_logger(), "[AUTO] %s", r.reason.c_str());
                            continue;
                        }
                        start_delay_sec_ = v;
                    }
                    else continue;

                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO] %s 변경 -> 현재 설정: %s / 대기 %.1f초 / %.1f초 / 추력 %.0f%% / 각도 %.1f도"
                        "  (**다음 AUTO 진입부터 적용**)",
                        n.c_str(), scenario_.c_str(), start_delay_sec_, run_sec_, throttle_pct_, angle_deg_);
                }
                return r;
            });

        RCLCPP_INFO(this->get_logger(),
            "=== [Auto Scenario] 시나리오 '%s' / 진입 후 대기 %.1f초 / %.1f초 / 추력 %.0f%% / 각도 %.1f도 ===",
            scenario_.c_str(), start_delay_sec_, run_sec_, throttle_pct_, angle_deg_);

        auto_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/auto/command", 10);

        // 대기/주행 상태. /auto/command 의 추력만 봐서는 "대기"와 "추력 0 시나리오(attitude)"를
        // 구별할 수 없어 따로 낸다. 매 틱 발행 — 구독자가 이 노드의 사망을 끊김으로 알아챈다.
        auto_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/auto/state", 10);

        // 진입 때 잠근 설정 + 회차. 숫자가 네 개뿐이라 커스텀 메시지를 만들지 않는 관례를 따른다.
        run_info_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/auto/run_info", 10);

        // btn1+btn2 동시 (EKF 가 조합을 판정해 보내준다) -> 시나리오 한 칸 전환
        scenario_next_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/ui/scenario_next", 10,
            [this](const std_msgs::msg::Empty::SharedPtr) { advance_scenario(); });

        // 자세 토픽은 모드 감별에만 쓴다 (제어각은 PID 노드가 쓴다)
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/filtered/attitude", 10, std::bind(&AutoScenarioNode::attitude_callback, this, std::placeholders::_1));

        loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&AutoScenarioNode::scenario_loop_callback, this));
    }

private:
    // "straight,dive" -> {"straight","dive"}. 모르는 이름은 버리고 경고한다.
    std::vector<std::string> parse_list(const std::string &csv) const {
        std::vector<std::string> out;
        std::stringstream ss(csv);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            tok.erase(0, tok.find_first_not_of(" \t"));
            const auto e = tok.find_last_not_of(" \t");
            if (e != std::string::npos) tok.erase(e + 1);
            if (tok.empty()) continue;
            if (!is_known_scenario(tok)) {
                RCLCPP_ERROR(this->get_logger(), "[AUTO] combo_list 의 '%s' 는 모르는 시나리오 — 건너뜁니다", tok.c_str());
                continue;
            }
            out.push_back(tok);
        }
        return out;
    }

    // 조합 누름 1회 = 목록에서 다음 시나리오. **모드는 건드리지 않는다.**
    void advance_scenario() {
        if (combo_list_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "[AUTO] combo_list 가 비어 전환할 수 없습니다");
            return;
        }
        combo_idx_ = (combo_idx_ + 1) % combo_list_.size();
        scenario_  = combo_list_[combo_idx_];
        RCLCPP_WARN(this->get_logger(), "======================================================");
        RCLCPP_WARN(this->get_logger(), "[AUTO] 시나리오 전환 -> '%s'  (%zu/%zu)  %.1f초 추력 %.0f%% 각도 %.1f도",
                    scenario_.c_str(), combo_idx_ + 1, combo_list_.size(),
                    run_sec_, throttle_pct_, angle_deg_);
        RCLCPP_WARN(this->get_logger(), "       btn1 짧게로 AUTO 에 들어가면 이 시나리오가 실행됩니다.");
        RCLCPP_WARN(this->get_logger(), "======================================================");
    }

    static bool is_known_scenario(const std::string &s) {
        return s == "straight" || s == "dive" || s == "climb"
            || s == "porpoise" || s == "attitude"
            || s == "bank_left" || s == "bank_right"
            || s == "static";
    }

    // ── 시나리오 정의 — 새 시나리오는 여기에 한 갈래 추가하면 끝난다 ──────
    std::vector<Step> build_steps() const {
        const double A = angle_deg_;
        // 칸 순서: {지속초, 롤, 피치, 추력%}
        if (scenario_ == "dive")     return {{run_sec_, 0.0,  A,   -1.0}};
        if (scenario_ == "climb")    return {{run_sec_, 0.0, -A,   -1.0}};
        if (scenario_ == "porpoise") return {{3.0, 0.0,  10.0, -1.0},
                                             {3.0, 0.0, -10.0, -1.0}};
        if (scenario_ == "attitude") return {{3.0, 0.0,  10.0,  0.0},
                                             {3.0, 0.0, -10.0,  0.0}};
        // 뱅크 선회 — 롤 칸에 각도, 피치 0. 롤 − = 왼쪽 (2026-09-16 실측)
        if (scenario_ == "bank_left")  return {{run_sec_, -A, 0.0, -1.0}};
        if (scenario_ == "bank_right") return {{run_sec_,  A, 0.0, -1.0}};
        // 완전 정지 — hold(서보 고정) + open_ended(run_sec 무시, MANUAL 까지 계속).
        // 롤/피치/추력 값은 PID 가 건너뛰므로 쓰이지 않고, sec 도 open_ended 라 쓰이지 않는다.
        if (scenario_ == "static")     return {{run_sec_, 0.0, 0.0, 0.0, true, true}};
        return {{run_sec_, 0.0, 0.0, -1.0}};    // straight (기본)
    }

    // 추력 % -> ESC PWM(us). 1000 = 정지, 2000 = 최대.
    static uint16_t pct_to_pwm(double pct) {
        if (pct < 0.0)   pct = 0.0;
        if (pct > 100.0) pct = 100.0;
        return static_cast<uint16_t>(1000.0 + pct * 10.0);
    }

    void attitude_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // roll 에 실린 ±5000 오프셋으로 모드 감별.
        // roll 실제 범위는 ±180도이므로 |x| > 2500 이면 오프셋이 실린 것 = 자동 모드.
        is_auto_mode_ = (msg->x > 2500.0f || msg->x < -2500.0f);
    }

    void scenario_loop_callback() {
        auto cmd_msg = geometry_msgs::msg::Quaternion();
        int  state   = AUTO_STATE_IDLE;   // 주행 종료 후·MANUAL 은 모두 IDLE

        // 1. 상태 전이(Rising Edge): 수동에서 자동으로 막 넘어온 순간.
        //    이 처리가 없으면 노드 기동 시각부터 세게 되어, 자동 모드로 들어가자마자
        //    시나리오가 이미 끝난 상태로 시작한다.
        if (is_auto_mode_ && !prev_auto_mode_) {
            auto_entry_time_ = this->now();
            run_started_     = false;
            run_done_logged_ = false;
            // **진입 순간의 설정을 통째로 잠근다.** 대기 중이든 달리는 도중이든 파라미터가
            // 바뀌어도 이번 주행은 진입 때 값으로 끝까지 간다 — 사후 분석에서 "무엇을
            // 지시했나"가 한 값으로 확정되게 하기 위해서다. 대기 시간도 같이 잠근다.
            active_scenario_  = scenario_;
            active_run_sec_   = run_sec_;
            active_thr_pct_   = throttle_pct_;
            active_delay_sec_ = start_delay_sec_;
            active_steps_     = build_steps();
            run_id_++;   // 회차. 진입마다 1 증가하고 노드가 사는 동안 유지된다
            // 지속 시나리오인가 — 첫 단계의 open_ended 로 판정한다(지속 시나리오는 한 줄짜리다)
            active_open_ended_ = !active_steps_.empty() && active_steps_.front().open_ended;

            double table_sec = 0.0;
            for (const auto &s : active_steps_) table_sec += s.sec;

            RCLCPP_WARN(this->get_logger(), "=========================================");
            if (active_open_ended_) {
                RCLCPP_WARN(this->get_logger(),
                    "[AUTO SCENARIO] AUTO 진입 — %.1f초 뒤 '%s' 시작  (MANUAL 로 나올 때까지 계속)",
                    active_delay_sec_, active_scenario_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[AUTO SCENARIO] AUTO 진입 — %.1f초 뒤 '%s' 시작  (%.1f초, 추력 %.0f%%)",
                    active_delay_sec_, active_scenario_.c_str(), active_run_sec_, active_thr_pct_);
            }
            // run_sec 을 보지 않는 시나리오에는 "뒷단계가 안 돈다" 경고가 무의미하다
            if (!active_open_ended_ && table_sec > active_run_sec_ + 1e-6) {
                RCLCPP_WARN(this->get_logger(),
                    "[AUTO SCENARIO] 주의: 단계표가 %.1f초인데 run_sec 은 %.1f초 — "
                    "뒷단계는 실행되지 않는다. run_sec 을 늘리세요.", table_sec, active_run_sec_);
            }
            RCLCPP_WARN(this->get_logger(), "=========================================");
        }

        if (is_auto_mode_) {
            // run_sec 은 **대기가 끝난 순간부터** 센다. 대기 중에는 elapsed 가 음수다.
            const double since_entry = (this->now() - auto_entry_time_).seconds();
            const double elapsed     = since_entry - active_delay_sec_;

            if (elapsed >= 0.0 && !run_started_) {
                run_started_ = true;
                RCLCPP_WARN(this->get_logger(), "=========================================");
                if (active_open_ended_) {
                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO SCENARIO] 시작! '%s' — btn1 으로 MANUAL 에 나올 때까지 계속 측정합니다",
                        active_scenario_.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO SCENARIO] 시작! '%s'  %.1f초  추력 %.0f%%",
                        active_scenario_.c_str(), active_run_sec_, active_thr_pct_);
                }
                RCLCPP_WARN(this->get_logger(), "=========================================");
            }

            if (elapsed < 0.0) {
                // 2. 시작 대기 — 추력 0, 수평. 물에 넣고 손을 뗄 시간을 준다.
                //    대기 중 btn1 로 MANUAL 에 나가면 취소되고, 다시 들어오면 처음부터 기다린다.
                cmd_msg.x = 0.0;
                cmd_msg.y = 0.0;
                cmd_msg.z = 0.0;
                cmd_msg.w = 1000.0;
                state     = AUTO_STATE_WAIT;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[AUTO SCENARIO] 대기 중 — '%s' 시작까지 %.0f초",
                    active_scenario_.c_str(), std::ceil(-elapsed));
            } else if (!active_steps_.empty() && (active_open_ended_ || elapsed < active_run_sec_)) {
                // 3. 단계표에서 지금 단계를 고른다. 표가 run_sec 보다 짧으면 되풀이한다.
                double table_sec = 0.0;
                for (const auto &s : active_steps_) table_sec += s.sec;
                double u = (table_sec > 0.0) ? std::fmod(elapsed, table_sec) : 0.0;

                const Step *cur = &active_steps_.back();
                for (const auto &s : active_steps_) {
                    if (u < s.sec) { cur = &s; break; }
                    u -= s.sec;
                }

                // 4. 단위 규약: x/y 는 0.1도 단위(PID 가 *0.1f 로 환산), w 는 PWM(us).
                //    z 는 서보 고정 플래그 (2026-09-16): 1 이면 PID 가 통째로 건너뛰고
                //    [1500,1500,1500,1000] 을 낸다. 요 목표가 아니다 — 요 제어는 없다.
                const double thr_pct = (cur->thr_pct < 0.0) ? active_thr_pct_ : cur->thr_pct;
                cmd_msg.x = cur->roll  * 10.0;
                cmd_msg.y = cur->pitch * 10.0;
                cmd_msg.z = cur->hold ? 1.0 : 0.0;
                cmd_msg.w = static_cast<double>(pct_to_pwm(thr_pct));
                state     = AUTO_STATE_RUN;

                if (cur->hold && active_open_ended_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[SCENARIO RUN] '%s' %.1f초 경과 — 완전 정지, 센서만 측정 중 "
                        "(btn1 으로 MANUAL 에 나오면 종료)",
                        active_scenario_.c_str(), elapsed);
                } else if (cur->hold) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[SCENARIO RUN] '%s' %.1f/%.1f초  완전 정지 — 서보 1500 고정, 추력 0 (센서만 측정)",
                        active_scenario_.c_str(), elapsed, active_run_sec_);
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[SCENARIO RUN] '%s' %.1f/%.1f초  롤 %.1f 피치 %.1f도  추력 %.0f%%",
                        active_scenario_.c_str(), elapsed, active_run_sec_,
                        cmd_msg.x * 0.1, cmd_msg.y * 0.1, thr_pct);
                }
            } else {
                // 5. 주행 종료 — 추력을 끊고 수평을 계속 지시한다.
                //    (자세 지시를 놓으면 목표가 사라지는 게 아니라 마지막 값이 남으므로
                //     명시적으로 수평을 계속 준다.)
                cmd_msg.x = 0.0;
                cmd_msg.y = 0.0;
                cmd_msg.z = 0.0;
                cmd_msg.w = 1000.0;
                if (!run_done_logged_) {
                    run_done_logged_ = true;
                    RCLCPP_WARN(this->get_logger(),
                        "[AUTO SCENARIO] '%s' %.1f초 주행 완료 — 추력 정지. "
                        "다시 달리려면 btn1 으로 MANUAL 로 나갔다가 AUTO 로 재진입하세요.",
                        active_scenario_.c_str(), active_run_sec_);
                }
            }

        } else {
            // [수동 모드]: 중립 대기.
            // 자동 모드로 전환되는 순간 직전 값이 남아있지 않도록 계속 중립을 덮어쓴다.
            cmd_msg.x = 0.0;
            cmd_msg.y = 0.0;
            cmd_msg.z = 0.0;
            cmd_msg.w = 1000.0;   // 스로틀 최소(정지)
        }

        prev_auto_mode_ = is_auto_mode_;
        auto_cmd_pub_->publish(cmd_msg);

        // 상태와 시나리오를 한 숫자로. MANUAL·주행종료(IDLE)에서는 **진입 때 잠근 값이
        // 아니라 지금 걸려 있는 설정**을 싣는다 — 다음에 btn1 을 누르면 무엇이 도는지를
        // 로그에서 미리 볼 수 있어야 하기 때문이다. WAIT/RUN 중에는 잠긴 값이 나간다.
        const int scen_id = (state == AUTO_STATE_IDLE) ? scenario_id_of(scenario_)
                                                       : scenario_id_of(active_scenario_);
        std_msgs::msg::Int32 st;
        st.data = 100 * scen_id + state;
        auto_state_pub_->publish(st);

        // 잠긴 설정 + 회차. **진입 때 잠근 값을 그대로 낸다** — 지금 파라미터가 아니라
        // "이번 주행이 무엇으로 돌았나"가 로그에 남아야 한다(설정은 주행 중에도 바뀔 수 있다).
        // 각도는 angle_deg_ 를 잠그지 않으므로(단계표에 이미 녹아 있다) 현재 값을 낸다.
        std_msgs::msg::Float32MultiArray ri;
        ri.data = { (float)run_id_, (float)active_thr_pct_, (float)angle_deg_, (float)active_run_sec_ };
        run_info_pub_->publish(ri);
    }

    bool is_auto_mode_;
    bool prev_auto_mode_;      // 상태 전이 판별용 과거 상태
    bool run_done_logged_;     // 종료 로그를 한 번만 찍기 위한 래치
    bool run_started_ = false; // 대기 끝 "시작!" 로그를 한 번만 찍기 위한 래치

    // 설정값 (파라미터, 언제든 바뀔 수 있다)
    std::string scenario_;
    double run_sec_, throttle_pct_, angle_deg_, start_delay_sec_;
    std::vector<std::string> combo_list_;   // 조합 누름이 순환할 시나리오 목록
    size_t combo_idx_ = 0;

    // 이번 주행에 잠긴 값 (진입 순간 복사)
    std::string active_scenario_;
    double active_run_sec_ = 3.0, active_thr_pct_ = 20.0, active_delay_sec_ = 5.0;
    bool   active_open_ended_ = false;   // run_sec 을 보지 않고 MANUAL 까지 계속하는 시나리오인가
    std::vector<Step> active_steps_;

    rclcpp::Time auto_entry_time_;   // AUTO 진입 시각 — 대기와 주행 시간 모두 여기서 센다

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr auto_cmd_pub_;
    int32_t run_id_ = 0;   // AUTO 진입 회차 (노드 기동 시 0, 진입마다 +1)
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr           auto_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr run_info_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr scenario_next_sub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoScenarioNode>());
    rclcpp::shutdown();
    return 0;
}
