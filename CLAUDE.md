# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Full measurement history — every experiment, retraction, and trap — lives in **`docs/imu_detailed.md`**
(Korean); "§N" below points into it. Its results are settled by measurement — do not reopen them without a new one.

## What this is

ROS2 Jazzy control software for an underwater fish robot, running on a Raspberry Pi 5 (Ubuntu Noble).
Single package `fish_robot_pkg`, 8 C++ nodes, no custom messages. The Pi does sensor fusion, PID
control, and logging; an **nRF52840 MCU** (firmware lives outside this repo) owns the RC link, the
IMU, and the servo/ESC PWM outputs.

## Commands

```bash
colcon build --packages-select fish_robot_pkg       # from workspace root
source install/setup.bash
ros2 launch fish_robot_pkg fish_robot.launch.py     # run everything
ros2 run fish_robot_pkg state_estimation_ekf_node   # run one node in isolation

# The launch normally runs as a systemd service (enabled + active on this machine).
sudo systemctl {status,restart,stop} fish-robot
journalctl -u fish-robot -f              # node stdout goes to journald, not a terminal
```

**Stop the service before launching manually** — otherwise two copies of every node fight over
`/dev/ttyAMA0`, the I2C bus, and GPIO 21 (`libgpiod` line request will throw and kill `rpm_driver_node`).

**There are no tests.** `package.xml` declares `ament_lint_auto`/`ament_lint_common` as test deps,
but `CMakeLists.txt` has no `BUILD_TESTING` block, so `colcon test` runs nothing. Verification is
done on hardware by reading topics (`ros2 topic echo`) and the CSV logs in `log_csv/`.

## Architecture

Everything is a **100 Hz (10 ms) loop**. Data flows one direction with no feedback except the motor
command returning to the MCU:

```
nRF52840 ──UART 46B──▶ uart_bridge_node ──▶ /raw/imu_6dof ─┐
                              ▲                /rc/command  │
                              │                /rc/status ──┤
                              │                             ▼
I2C sensors ──▶ i2c_driver_node ──▶ /raw/magnetometer ──▶ state_estimation_ekf_node
                        │                                            │
                        │                                            ▼
                        │                                  /filtered/attitude
                        │                                    │      │       │
                        │        ┌───────────────────────────┘      │       └──┐
                        │        ▼                                  ▼          │
                        │ auto_scenario_node ─/auto/command─▶ pid_control_node  │
                        │                                           │          │
                        │        └─────────/motor/output────────────┘          │
                        │                       │                              │
                        │             uart_bridge_node ─UART 10B─▶ nRF52840     │
                        │                                                      ▼
                        └─ /sensor/pressure_raw ──▶ hydro_estimator_node ─▶ data_logger_node
                                    (+ /rc/status)      /filtered/hydro            │
                                                        /sensor/pressure_calibrated   ▼
                                                                            log_csv/*.csv
```

`rpm_driver_node` is independent: GPIO 21 rising-edge interrupts → `/sensor/tail_rpm`.
Not drawn: `auto_scenario_node` also publishes **`/auto/state`** (Int32, 0 IDLE / 1 WAIT / 2 RUN, 100 Hz,
since 2026-09-16), read by `hydro_estimator_node` (wait-window wet zero) and `data_logger_node` (`State` column);
`hydro_estimator_node` additionally reads `/raw/imu_6dof` for the stillness test. `i2c_driver_node` → `/raw/imu_mpu9250`
(second IMU, monitoring) → `data_logger_node` only.

### Node roles

| Node | Owns |
|---|---|
| `uart_bridge_node` | `/dev/ttyAMA0` 115200 8N1. Byte-level packet state machine. **Pure pass-through — no scaling, no sign flips.** |
| `i2c_driver_node` | `/dev/i2c-1`: AK8963 magnetometer (via MPU9250 bypass) + 3× MS5837 behind a TCA9548A mux. **Since 2026-09-16 it also reads the MPU9250's own 6-axis + chip temperature** → `/raw/imu_mpu9250` (Float32MultiArray[7]: accel g, gyro °/s, °C; ±4 g / ±500 °/s, DLPF 41 Hz; **rotated into the nRF IMU's chip axes** by the measured signed permutation `nRF = (−MPU_y, +MPU_z, −MPU_x)` so CSV columns 24–29 and 56–61 subtract directly — **monitoring only**, nothing but `data_logger_node` reads it; failed read = NaN×7; WHO_AM_I probed once at init like the AK8963) |
| `state_estimation_ekf_node` | MEKF filter, button UI, yaw offset, ±5000 mode encoding, mag calibration via `magneto_cal.py`. The "brain" — sole publisher of `/filtered/attitude`. **Pressure left this node on 2026-08-21** — it never entered the filter state; it was inherited furniture from the Mahony migration. |
| `pid_control_node` | PID + motor mixing. Output `[0]=left servo, [1]=right servo, [2]=yaw servo, [3]=tail BLDC`. **No yaw control since 2026-09-14 (operator decision)**: `[2]` is fixed at 1500 (center), the yaw stick is ignored, and the `kp/ki/kd_yaw` and `servo_reverse_yaw` parameters no longer exist. **`/auto/command` z is no longer dead — since 2026-09-16 it is the servo-hold flag** (≥ 0.5 in AUTO ⇒ skip the PID, emit `[1500,1500,1500,1000]`, and clear the integral/derivative state so the freeze cannot wind up); it is not a yaw target, and MANUAL ignores it. Roll/pitch mixing never used yaw, so the fins are unaffected. |
| `auto_scenario_node` | Time-based trajectory generator for AUTO mode. A scenario is a **step table** (duration + roll/pitch/thrust per step); adding one means adding a table in `build_steps()`. Eight exist: `straight` (default — level, 40 %, 3 s since 2026-09-16), `dive`, `climb`, `porpoise`, `attitude` (thrust 0, bench servo check), `static` (2026-09-16 — **full stop**: sets the `hold` flag on its Step, which becomes `/auto/command` z = 1 and makes `pid_control_node` skip the PID entirely and emit `[1500,1500,1500,1000]`; thrust 0 + level targets are NOT enough, because any tilt still produces an error the PID acts on — measured under a 10° tilt: `straight` drove the servos to 1412, `static` held 1500. For sensor-only baselines: depth drift, gyro bias, and validating the wait-window wet zero, where `Speed_wz` must read ~0. **Since 2026-10-01 it is also the one open-ended scenario**: its Step sets `open_ended`, so the run loop skips the `run_sec` test entirely and it keeps measuring until btn1 returns to MANUAL — pre-deciding a measurement length and editing a parameter per run did not match how it is actually used (09-30 operator run). The measured stretch is the CSV rows with `State` = 2), and `bank_left` / `bank_right` (roll ∓`angle_deg`, pitch 0, thrust on — bank turn with the yaw servo still fixed; roll sign bench-measured 2026-09-16: **− = left, + = right**; whether/which way it actually turns is unverified in water — read the turn from CSV `Yaw`, + = left/CCW). **`turn` was removed with yaw control on 2026-09-14** — naming it is rejected as unknown, and `/auto/command` z carries the servo-hold flag instead of a yaw target. **`scenario` has flipped between measurement and driving several times on operator request** — `static` 2026-10-01, `straight` 10-03, `static` 10-05, **`dive` 2026-10-14 (current)**, so a bare btn1 entry dives with no button press. Only that one default string moves; no scenario code changed any time. `combo_list` is a separate default, trimmed to **`"dive,straight"`** on 2026-10-14 (operator request) so the transmitter combo toggles dive against level-run and nothing else; bank turns are still in the code, just not on the button. Note the combo advances *past* the current slot, so the first press from the `dive` default lands on `straight` and the next returns to `dive`. Drive again with `ros2 param set /auto_scenario_node scenario straight`. Selected by the `scenario` parameter with `run_sec` / `throttle_pct` / `angle_deg` / `start_delay_sec`; **all are latched at AUTO entry**, so a param change applies on the next entry, not mid-run or mid-wait. **AUTO entry waits `start_delay_sec` (added 2026-09-15; default raised 5 → 10 s on 2026-09-19 so the wet zero gets a still stretch — the robot spends the first 4–5 s sinking) before the scenario starts** — thrust 0 and level during the wait, `run_sec` counts from the end of the wait, and leaving AUTO during the wait cancels it. Every scenario cuts thrust at `run_sec` **except an open-ended one** (`open_ended` on its Step — only `static` today), which ignores `run_sec` and ends when the operator leaves AUTO. Publishes `/auto/state` every tick (0 IDLE incl. MANUAL and after `run_sec`, 1 during the wait, 2 while running). |
| `rpm_driver_node` | Hall-sensor pulse counting in a worker thread |
| `hydro_estimator_node` | **Owns pressure end to end** (new 2026-08-21): atmospheric zeroing, `/sensor/pressure_calibrated`, btn1 long-press re-zero, and depth in metres on `/filtered/hydro`. Speed (pitot) is implemented (air-Δb or `/hydro/zero_q` zero, LPF, deadband) but **the air zero is void once wet** — channels drift 0.3–0.6 mbar apart, leaving +0.3–0.4 m/s at rest (0913–0915 tank logs). Hence the **wait-window wet zero (2026-09-16)**: during the AUTO start delay (`/auto/state` = 1) it collects `q_raw − dP_hydro`, tests stillness per sample (1 s depth-rate regression < 5 mm/s AND raw \|gyro\| 1 s mean < **7** °/s AND static gauge ≥ **3 mbar**), and at the WAIT→RUN edge takes the median of the **latest** ≥ 2 s still run (src 1), else the last 2 s (src 2), else zero 0 (src 0). Published as slots [8..14] of `/filtered/hydro` (`q_wz`, `speed_wz` unfiltered, per-sample); the old `speed`/`q` slots are untouched for comparison. Thresholds `wet_zero_*` are live-settable (latched at WAIT entry). Observation-only; no control loop consumes it.
**Verified on 2026-10-29 against 10-19…10-26 driving logs — 95 runs, zero `src=0`** (42 src 1 / 53 src 2), and in `static` (servos and motor dead, so true speed is 0) a src-1 run reads `Speed_wz` **−0.017 m/s** mean: the "+0.3–0.4 m/s once wet" bias really is removed. src 2 is measurably worse (−0.073 mean, 2.3× the range), which is what the `Zero_src` column is for. The two thresholds were then set from 100 replayed WAIT windows: **gyro 5 → 7 °/s** lifts src 1 from 43 % to 68 % while moving already-good zeros by only 0.0098 mbar (1/20 of sensor noise) — 5.0 was below the 0915 measurement (still-floating 6.2–6.5 °/s) it was derived from, and 8.0 buys +4 % while nearing the lift-out band (8–10). The **gauge ≥ 3 mbar** condition (`wet_zero_min_gauge_mbar`, ≤ 0 disables) closes a structural hole: air held steady passes the other two tests *better* than water, and in one 10-24 window the robot was lifted out and re-immersed mid-wait, so gyro 7 alone picked the **pre-immersion air** stretch and the zero jumped 2.46 mbar (≈0.7 m/s). It costs 2 of those 68 windows and nothing else. Since the transmitter link dies underwater, the operator presses btn1 and *then* lowers the robot — "already wet during the wait" cannot be assumed. |
| `data_logger_node` | 73-column CSV snapshot at 100 Hz (47 before 2026-09-16; then 8 wet-zero fields + `State`, 7 MPU9250 columns `MPU_*`, and since 2026-10-07 `Epoch(s)` — the UTC unix time of each row, ms precision, read straight from `CLOCK_REALTIME`. `Time(s)` stays the monotonic logger-start clock and is what every duration/rate calculation must use; `Epoch(s)` exists only to put the log on the same timeline as gear outside the robot (e.g. a tow rig whose filename carries `YYYYMMDD_HHMMSS`), since a wall clock can step backwards on an NTP correction. `Epoch − Time(s)` is constant across the 200 MB rotation (measured jitter 0.0 ms), which filename-based back-calculation is not. Column 64 `Scenario` since 2026-10-14 — the scenario id from `/auto/state`, so a log says what ran without digging through the journal; in MANUAL and after `run_sec` it holds what *would* run next, so filter on `State`=2 for what actually did. Columns 65–73 since 2026-10-15 put the run's identity and settings on every row — `Run_id` (incremented at each AUTO entry, 0 at node start), the latched `Throttle_pct`/`Run_sec` plus the live `Angle_deg`, the live `Kp_pitch`/`Pitch_OL` from the new `/pid/debug`, the `Target_Pitch`/`Target_Roll` actually commanded, and `VBat` (the `/rc/status` centivolts the logger used to discard, divided by 100). `/pid/debug` is not published on the failsafe or `static` early-return paths, so 69–72 hold the previous value there) — fused attitude **plus raw IMU/mag/pressure and EKF gyro bias + flags** (post-hoc diagnosis; raw mag enables offline recalibration, raw pressure enables offline depth/pitot recomputation). 200 MB per-file rotation, `Time(s)` continuous across files. Schema: `docs/csv_format.md`. |

## Conventions you must know before editing

These are load-bearing and span multiple files. Breaking one silently corrupts behavior.

### 1. Mode is smuggled inside the attitude message

There is no mode topic. `state_estimation_ekf_node` adds **±5000 to `attitude.x` (roll)** in AUTO
mode. Three nodes decode it independently with the same `|x| > 2500` test — `attitude_callback` in
`pid_control_node.cpp`, `auto_scenario_node.cpp`, `data_logger_node.cpp`. Change the encoding in
one place and you must change all four.
Since 2026-09-16 there is also an explicit **`/auto/state`** Int32 from `auto_scenario_node`, and
since 2026-10-14 it carries two facts in one number: **`100 * scenario_id + state`** (state 0 IDLE /
1 WAIT / 2 RUN; ids in `SCEN_*`, 0 = unknown). `hydro_estimator_node` must take **`% 100`** — without
it the value reads 802 and the WAIT→RUN edge is never seen, so the wet zero silently never captures.
`data_logger_node` splits it into the `State` and `Scenario` columns. Scenario ids are written into
past logs, so **never renumber them**; adding one means touching the `SCEN_*` enum, the logger's
decode, and the table in `docs/csv_format.md` together.

### 2. IMU units are **g** and **deg/s**, not REP-103

`/raw/imu_6dof` is a `sensor_msgs/Imu`, but the nRF firmware sends **g** and **deg/s**, published
unconverted (measured |a| ≈ 1.03 at rest, not 9.81; the estimator multiplies the gyro by `M_PI/180`
itself). The filters normalize accel, so scale doesn't
matter to them, only direction. **Any new consumer of this topic must convert.**

### 3. Message types are reused as plain containers

- `/rc/command` is a `geometry_msgs/Quaternion` holding **4 raw stick ints**, not a rotation:
  `x=roll, y=pitch, z=yaw, w=throttle`. Same for `/auto/command` (`x/y` in **0.1°**, `w` in **PWM µs**;
  **`z` is the servo-hold flag since 2026-09-16**, 0 normally and 1 for the `static` scenario — not an angle).
  Measured full-stick ranges (2026-08-27): roll −272~+242, pitch −266~+251, **yaw only −85~+86**
  (1/3 the scale of roll/pitch — nRF-side mapping), throttle 0~+1171 (the code's 1171 constant is
  exact). Roll/pitch sticks map to target angle at ×0.1°/count in `pid_control_node`. **The yaw
  stick is ignored entirely since 2026-09-14** (operator decision — yaw servo fixed at center); it is
  still logged raw as `RC_Y`. Before that, full yaw stick commanded only a ±7° heading target.
- `/rc/status` is `Int32MultiArray[5]` = `[btn1, btn2, vbat1×100, vbat2×100, rssi]`. Voltages are
  centivolts — divide by 100.
- `/sensor/pressure_raw` is `Float32MultiArray[6]`: `[0..2]` pressure (mbar), `[3..5]` temperature (°C).
  **A dead channel is `NaN`, never `0.0`** (changed 2026-08-20, same on `/sensor/pressure_calibrated`).
  On the zeroed topic `0.0` is a legitimate reading — *at the surface* — so the old `0.0` sentinel made
  "bus is dead" and "floating at the surface" the same value, and a depth loop would keep diving on a
  locked bus. NaN collides with nothing, still fails the existing `>= 100 mbar` validity test (every
  comparison with NaN is false, so downstream needed no change), and propagates through arithmetic so
  a consumer that forgets to check breaks loudly instead of quietly. **Test with `std::isnan(x)` —
  `x == NAN` is always false.** `i2c_driver_node` also logs an ERROR after 30 consecutive all-channel
  failures (~3 s), because the mux-select and ADC-read failure paths used to be silent.

### 4. `-9999` is the RC-link-lost sentinel

The nRF sends `throttle = -9999` when the transmitter link drops. `pid_control_node` treats it two
ways — the explicit sentinel, and a 5 s timeout on `last_valid_rc_time_` (for when the nRF itself
dies). `data_logger_node` records it unfiltered so dropouts stay visible in the CSV. **The failsafe
is skipped entirely in AUTO mode** (`!is_auto_mode_ &&`) by design — autonomous running without a
transmitter is the normal case.

### 5. The control loop is event-driven, not timed

`pid_control_node` has **no timer** — PID runs inside `attitude_callback`, and it **omits `dt`**
(gains absorb it, assuming a fixed 100 Hz). `/filtered/attitude` must only ever have ONE publisher
(ghost-publisher check below) — but be precise about *why*, because the obvious reason is currently
false: **`ki` and `kd` all default to `0.0`** (since 2026-08-26 the nine gains are ROS parameters,
live-settable via `ros2 param set /pid_control_node kp_pitch <v>`; changing a `ki` resets that
axis's integral accumulator, because it accumulates even while `ki`=0), so P-only control is
rate-independent and doubling the attitude rate does **not** change the servo command. The real
damage from two publishers is (1) two different estimators' attitudes arriving alternately, so the
output chatters between two values, and (2) `/motor/output` doubling to 200 Hz, which doubles the
UART packet rate to the nRF. The `dt`-omission gain problem becomes real **the moment I or D is
enabled** — which is exactly when someone will least expect it. Telling an operator "the gains are
wrong" today sends them to re-tune gains that are fine.
`state_estimation_ekf_node` measures `dt` per sample from `header.stamp` and integrates it with no
upper clamp (covariance adjusts trust). §1.3.

### 6. Startup is not instantaneous

- **The gyro-bias stillness window is 1000 samples (~10 s), but it does NOT block.** The EKF
  initialises on 50 samples (~0.5 s) and publishes attitude from then on; the 1000-sample window runs
  **in parallel** and joins as a pseudo-measurement when it closes (`state_estimation_ekf_node.cpp:1245`).
  The robot must still be completely still for those 10 s — motion trips the stillness test
  (`자이로 std < 1.0 °/s` and `|a|` deviation `< 0.05 g`) and restarts the window, up to 3 times.
  The old "blocks for 10 s, nothing publishes" behaviour was the **retired Mahony node's**.
- **Pressure zeroing waits 180 s** (`atm_warmup_sec` in `hydro_estimator_node`, cut from 300 s on
  2026-08-20) before capturing baseline. It delays **depth only** — pitot speed needs no atmospheric
  zero (the per-channel offsets enter the differential as a constant and are absorbed by the in-water
  q zero), so it will publish from t=0 once implemented. The 300 s figure sized a 30BA-era steep initial drop (~250 s); with 02BA
  sensors **that transient is absent** — slopes sit inside the estimator's own noise floor from
  t=30 s, and 180 s vs 300 s changes the resulting depth error by 0.03–0.17 mm, below sensor noise.
  Re-verify after final assembly (`press_char.py --s1`); a sealed housing may bring the transient back.
  Re-zero any time with a btn1 long press (**in air** — the node refuses above 5 mbar gauge, because a
  shallow submersion passes the `[900,1100]` plausibility gate and would silently bake that depth into
  the zero). **A refused or failed re-capture never destroys the zero in use** — new per-channel offsets
  are staged and only committed on success, so depth never drops out mid-dive.

### 7. Hardware quirks

- **`gpiochip4`**, not `gpiochip0` — the Pi 5 moved the 40-pin header onto the RP1 chip. Changing
  boards means changing `rpm_driver_node.cpp`.
- RPM resolution is coarse: 1 pulse per 100 ms window = **150 RPM** (1 rev = 4 hall pulses,
  measured 2026-08-27; the old "3 pulses/rev" spec assumption read 33 % high). Hand-turn encoder
  tests need the motor battery connected — without it ~2/3 of pulses are missed.
- The three pressure sensors are **MS5837-02BA** (swapped from 30BA on 2026-08-20). The two models
  need **different compensation constants** — using 30BA math on an 02BA reads exactly **20× high**
  (measured: 20132.60 vs 1006.63 mbar) and sails past the estimator's `>= 100 mbar` validity test,
  silently scaling depth by 20. `i2c_driver_node.cpp` warns when pressure leaves 300–1200 mbar to
  catch this. 02BA vs 30BA on paper: resolution 0.016 vs 0.20 mbar (0.16 mm vs 2 mm), full-accuracy
  depth **~1.9 m** vs ~300 m, and the 02BA datasheet expects drying roughly once a day.
- **The 02BA's 0.016 mbar resolution does NOT hold on this hardware** (measured 2026-08-20, 60 min,
  filter bypassed): σ = 0.13 / 0.13 / 0.24 mbar (**1.3 / 1.3 / 2.4 mm**) on ch0/ch1/ch2 — **8–15×
  the datasheet**. The three channels' fluctuations are uncorrelated (r ≈ −0.005, and averaging the
  three drops σ by exactly √3), so it is **per-sensor electrical noise**, not real pressure, supply
  ripple, or EMI — those would be common-mode. Still 5–10× finer than the cm-level a depth loop
  needs, but it caps differential-pressure attitude: at 30 cm spacing 1° of pitch is only 0.51 mbar,
  so a single sample resolves ~0.3–0.5°. Quote the measured figure, never the datasheet one.
- The three MS5837 sensors all share address `0x76`, hence the mux. Pressure updates at **~19.3 Hz**
  (was ~11 Hz until 2026-09-19) despite the 100 Hz timer — a 3-state machine spreads the ADC
  conversions across ticks, and the three sensors convert concurrently so the rate is independent of
  channel count. Two changes bought the 1.75×, both **without touching OSR 8192 or the compensation**:
  (1) **temperature is converted only every 10th cycle** (`TEMP_EVERY`), the D2 in between being the
  last *successful* one — measured drift is 0.00–0.06 °C/min, i.e. 0.0005 °C over 0.5 s against a
  0.01 °C LSB, while that second 40 ms wait was half the period; (2) the per-conversion wait went
  40 → 30 ms (`CONV_TICKS=3`) against a 17.2 ms datasheet max, still 1.7× headroom. Noise is unchanged
  (0.18/0.22/0.20 mbar at 19.3 Hz vs 0.14/0.25/0.21 at 11 Hz — same order, within per-unit spread).
  **Raising the sample rate by lowering OSR was tested and rejected (2026-10-28).** With the period
  held fixed (`CONV_TICKS=3`, 19.30 Hz both runs, zero dropouts), dropping D1 from OSR 8192 to 2048
  made the per-sample σ **2.01× worse** (ch0 2.08 / ch1 2.01 / ch2 1.95) — matching the datasheet's
  ~2.0× ratio. So the per-sample noise is dominated by the chip's own oversampling, and the 9–12×
  excess over datasheet does **not** come from outside the chip; trading OSR for more samples buys
  nothing after averaging. Keep 8192. Separately, the **1-second average is not limited by per-sample
  noise at all**: averaging 19 samples should cut σ by √19.3 = 4.4× but only cuts it 2.2–3.7× (0-stage
  baseline), and the 1 s SD actually *fell* at OSR 2048 — a low-frequency component dominates there,
  which neither OSR nor sample rate touches. Anything averaged at ~1 s (depth, speed) is capped by
  that, not by OSR.
  **A skipped cycle reuses D2 only when `d2_ok[]` says the last collection succeeded**; a failed one
  is zeroed so the conversion guard emits NaN — otherwise this would resurrect the "dead sensor keeps
  publishing its last value" trap. `hydro_estimator_node` hardcoded 11 Hz in five places (LPF `dt`,
  deadband σ, q-window sizing, collection-time logs, the 5 s negative-q streak); they now read one
  `PRESS_HZ` constant — **change the driver and you must change that too**, or the LPF time constant
  and deadband threshold shift silently. The mux stagger between the three (~0.8 ms at 100 kHz) is the
  smallest delay in the chain by 100×.
- **`/sensor/pressure_raw` is truly raw — permanently, by decision (2026-08-26).** The driver's
  1st-order IIR (α 0.1, τ ≈ 0.86 s) was bypassed on 2026-08-19 for characterization and the bypass
  is now the spec: restoring it would insert ~860 ms of unmodeled group delay under
  `hydro_estimator_node`'s 72 ms attitude alignment, double-filter a q path whose LPF/deadband/swap
  thresholds are designed around the **raw** σ (0.189 mbar), and destroy the raw CSV record that
  post-hoc analysis depends on (the 8–15× noise discovery required the bypass). Consumers filter
  for themselves; a future depth loop picks its own coefficient in its own node.
- A pressure channel that fails at **PROM load** is marked `prom_C_[i][1] == 0` and skipped
  entirely; its slot publishes NaN (not `0.0` — see §3). But **PROM success does not mean the
  sensor works**: two units (C1 42722 on 2026-09-02, C1 43114 on 2026-09-04) answered reset and
  returned a CRC-valid PROM while **NACKing every ADC conversion command (0x40–0x4A) at every
  OSR**. That channel publishes NaN with **no journal trace at all**, because
  `i2c_driver_node`'s failure ERROR only fires when *all three* channels fail
  (`press_fail_streak_`). `mux_probe.py` used to call this "정상"; since 2026-09-04 it runs a
  third stage that converts an actual pressure, and reports `ADC 사망`. Sensor vs connector is
  settled by swapping the suspect into a known-good channel.
  **All three units were replaced on 2026-09-08** — now C1 41899 / 43494 / 42949 on ch0/1/2, every
  ADC alive for the first time, roles re-verified by blow test (`config/port_map.txt`).
- `SERVO_MIN_US`/`SERVO_MAX_US` (1325/1675 since 2026-08-26 — horn ≈ ±20° on the HS-5086WP's
  0.114°/µs) in `pid_control_node.cpp` **must stay inside the nRF firmware's `MotorControl.cpp`
  window (1250/1750)** — narrower here is safe (the nRF clamp never triggers); wider is not.
  This repo cannot enforce that.

### 8. Magnetometer: fallback, axes, calibration

- **Failure is a supported state**: 500 ms of AK8963 silence → automatic 9-DOF → 6-DOF fallback. It
  is alive (~94 Hz since 2026-07-31), so the system runs 9-DOF; yaw drift there is +0.04°/min. The
  −8°/min figure is 6-DOF-only, and no 6-DOF drift number survives a session (§3.2, §6.3).
  **A boot with the IMU unreachable stays 6-DOF for that whole run** (2026-09-08): `i2c_driver_node`
  probes MPU9250 (0x68, bypass) and AK8963 once at init and never retries, so a loose **J4**
  connector (disturbed during the pressure-sensor swap) gives one boot-time WARN and silent 6-DOF.
  Diagnostic: `i2cget -y 1 0x68 0x75` must answer `0x71`; fix = reseat J4, then restart the node.
  The MPU9250 6-axis monitor (2026-09-16) sits on the same J4 and follows the same once-at-init probe,
  so a loose J4 also means `MPU_*` columns stay NaN for that whole run.
- **Axis remap `(my, mx, −mz)` is verified and PERMANENT** (tumble 2026-08-06, dip 144.3° vs Korea's
  143°); the code comment's derivation is bogus but the answer is measured — do not reopen (§4.2).
- **Ellipsoid fit only; min/max retired 2026-08-17** — it measured *worse than no calibration*
  (38.1 % vs 22.1 % scatter). btn2 long press and `/calibrate_mag` (`ros2 service call
  /calibrate_mag std_srvs/srv/Trigger` — call twice: start, then finish early) both run
  `magneto_cal.py`: refuses to save above 8 % scatter, backs up the old file, and the EKF node
  auto-reloads on success, no restart (§4.3, §9).
- **Calibration is tied to the ASSEMBLY, not the place** — any part change invalidates it.
  Recalibrated 2026-08-25 (pressure assembly): **scatter 3.4 %, dip 140.3° vs 143 expected** — the
  best yet; hard iron (−7.5, +20.5, −83.8) µT, back to chip-Z dominance like the 08-06 strip state.
  Supersedes 2026-08-17 assembled (6.1 %, 137.2°, (+9.9, +61.6, −16.1)) (§9, §10.2). **INVALID since
  2026-09-08**: a `magneto_cal.py` run on 2026-09-02 16:08 wrote a diverged fit (bias 7420 / 1422.7 /
  1357.12 µT, scale 1/1/1 — the EKF dip check reads 70°) and it was swept into a docs commit unnoticed;
  the 08-25 values were restored from git on 09-08, but they no longer fit the current assembly either
  (dip 175.5°, |m| 105 µT — the −84 µT chip-Z hard-iron source moved with the sensor swap).
  **Recalibrate before trusting yaw**; until then the mag gate rejects every sample and the EKF is
  effectively 6-DOF (§12.4). Tumble in one spot,
  ≥1 m from steel; **never calibrate from a flat spin** (it cannot see the vertical offset). File:
  **`~/ros2_ws/config/mag_calib_params.txt`** — moved out of `log_csv/` on 2026-08-25 after that file
  went missing in a log cleanup and the EKF spent an unknown period on source-code defaults (dip 11.1°,
  physically impossible; it printed an ERROR every boot and nobody read it). See `config/README.md`. (A stale `mag_calib.txt` sat at the workspace root,
  read by nothing — deleted 2026-08-25. It held June-9 **min/max** values, the method retired for
  measuring *worse than no calibration*, so it looked restorable but would have made things worse.)
- **A bad mag vector corrupted Mahony's roll/pitch** (5.0°/1.7° measured), not just yaw; the EKF's
  1-D horizontal yaw update is structurally immune (§5.1, §5.2).
- **The dip reference is bootstrapped from the first mag sample at boot, wherever the robot sits**
  (2026-09-14). The absolute check vs 143° only logs (ERROR above 30°) — the EKF keeps the wrong
  reference and its ±15° gate then **rejects the correct readings** once the robot is moved: the 20:17
  boot on a disturbed spot (|m| 67.9 µT vs ~38 clean) set 175.8°, and the rejected samples' median was
  144.3°. Boot the service away from steel/magnets and run `scripts/dip_check.py` before experiments;
  a wrong reference is fixed by restarting on a clean spot, not by recalibrating. The same day's tumble
  on that spot read dip 168.5°; one clean boot then gave 142.1°, but **on 2026-09-15 the dip turned out
  pose-dependent** — upright 139°, lying right side down 122–124°, left side down 140–145° — and the EKF
  rejected the mag for minutes while the robot lay still on its right side. A good calibration holds the
  dip within a few degrees in every pose, so that 09-14 calibration is not good: recalibrate on a clean
  spot. A single clean-boot dip cannot validate a calibration; check several poses with `dip_check.py`.
- **A hand-held spin cannot validate heading better than ~3°** — the residual sine is the ROOM
  (spatial field gradient), not the robot, and the calibration model is at its limit: do not re-fit
  it over a ~3° residual (§4.4, §4.6).

### 9. Button UI (`/rc/status`, 1 tick = 10 ms)

Short press = 30 ms–1 s; long press = exactly 3 s (`== 300`, fires once).

| | Short | Long |
|---|---|---|
| btn1 | AUTO/MANUAL toggle | re-zero atmospheric pressure, **in air only** — refused above 5 mbar gauge (**handled by `hydro_estimator_node`** since 2026-08-21; the EKF still detects the 3 s hold, but only to suppress the release from counting as a short press). The in-water q zero is deliberately *not* on this button; call `/hydro/zero_q` for it |
| btn2 | set current heading as yaw 0 (published attitude only — nothing steers on yaw since 2026-09-14) | run `magneto_cal.py` (ellipsoid); long-press again = finish collection early (SIGINT) |

**btn1+btn2 held together for 3 s = advance the AUTO scenario** (2026-09-03). While either button is
held, if the other is down on any tick the press is latched as a *combo*: every single-button action is
suppressed (mode toggle, heading zero, mag calibration, and `hydro_estimator_node`'s atmospheric
re-zero). **The scenario only advances at exactly 300 ticks of both being down**, publishing
`/ui/scenario_next`; a shorter combo does nothing but still suppresses, and logs how long it was held
(report the *peak* tick count, not the live counter — the counter resets the instant either button
lifts, which made a 2 s press report "0.0초"). `auto_scenario_node` steps through `combo_list`
(default `dive,straight` since 2026-10-14; was `straight,dive,bank_left,bank_right`, and pinned to `"static"` alone 2026-10-01 to 10-03) on that signal. **The latch clears only when BOTH are released** — the two
releases were measured 0.1 s apart, and clearing on the first would let the later one fire its own
action. Combo detection lives in `state_estimation_ekf_node` (which must suppress anyway) plus a
small guard in `hydro_estimator_node`; don't scatter a third copy. The combo does **not** change mode —
it only arms the scenario, which runs on the next plain btn1 entry. Simultaneous reporting by the
transmitter/nRF was verified 2026-09-03 (3/3), and the whole path end to end the same day.

When the RC link drops the nRF sends `btn = 255`; both `state_estimation_ekf_node` and
`data_logger_node` explicitly reject non-0/1 values so the `1 → 255` transition isn't misread as a release.

## Before any measurement: the ghost-publisher check

The costliest trap so far (§7.1): an orphaned estimator — ppid 1, running a *deleted* binary —
survived restarts, hid from `ros2 node list` and `pgrep -x` (comm truncates at 15 chars), and
blended two filters on one topic into a stable-looking fake 6.8° error. Before trusting any run:

```bash
ros2 topic info /filtered/attitude --verbose | grep -c "Endpoint type: PUBLISHER"   # must print 1
ls -l /proc/*/exe 2>/dev/null | grep '(deleted)'                                    # must be empty
```

## Closed questions — do not re-litigate (details in docs/imu_detailed.md)

- **IMU axis transform verified end to end** (`R = [0 0 −1; −1 0 0; 0 1 0]`, `det = +1`; accel
  2026-08-04, gyro 2026-08-06). Judge axis tests on at-rest residuals only, and stillness on the
  gravity vector, never on Euler angles (degenerate near pitch ±90) — §2.
- **halfwy bug** (`R22` where `R21` belongs, 9-DOF Mahony) fixed 2026-08-06. Its symptom grew as the
  calibration improved — never read "recalibration made attitude worse" as "the calibration is bad" (§5.1).
- **"9-DOF Mahony has a structural ~6.8° tilt error" RETRACTED** — a ghost-publisher artifact; clean
  re-measure 0.134°/0.117°. Do not apply the proposed `halfv` projection fix on theory alone (§7.1).
- **EKF accel update must project world-vertical out of the Kalman GAIN** (`null_dir = &g_b`) or yaw
  leaks −33°/−37° per shake; never pass `null_dir` to the mag or bias updates. Diagnostic: 6-DOF yaw
  1σ may only grow — score any decrease cumulatively, never per-sample (§5.2).
- **EKF vs Mahony**: tied at rest; EKF 2.8× better under rotate+shake, and passed the 6-DOF
  yaw-drift gate at +0.155°/min (≤ 2 °/min required) — §6.
- **Accelerometer needs no calibration** — axis radii agree to 0.3 %; the earlier "~2 % asymmetry"
  claim was an underdetermined-fit artifact (§7.3).

## What to do next

Working checklist, ordered so each item unblocks the next: **`docs/todo.md`**.
It carries the water-vs-bench split, the "must be done before water" items, and the open decisions.

## Known gaps (open)

| Gap | Notes |
|---|---|
| AK8963 ASA ROM (0x10–0x12) never read | \|m\| reads 30 % low (35.8 vs ~50 µT, 2026-08-25). Magnitude-only — heading/attitude unaffected; fixing it forces recalibration. §4.1 |
| Thermal drift near the sensor | 3 µT (≈8° heading) after CPU load, noise ~×3 — may cap accuracy. §4.7 |
| Gyro z bias session-unstable | −0.044 vs −2.2 °/s hours apart (50×); the EKF absorbs it online. §3.2. **Since 2026-09-16 the MPU9250's own gyro + temperature are logged (`MPU_G*`, `MPU_T`) as an independent reference** — same true rotation, independent bias, so a widening difference marks a bias jump. The driver already rotates it into the nRF chip axes (permutation measured 2026-09-16 by pairing the two gravity vectors at rest: det +1, residual 0.96°; the gyro uses the accel-derived permutation — not yet confirmed by a rotation test; re-measure if the nRF IMU is remounted). Bench at rest 2026-09-16, nRF axes: nRF (+0.15, −3.16, −0.50) vs MPU (+2.51, −0.33, −1.07) °/s, MPU 38 °C. |
| `gyro_noise_sigma` validated 2026-08-17 | Allan (2 h, 711k samples): ARW 0.0060–0.0078 °/s/√Hz, bias instability 0.0011–0.0024 °/s at τ 69–434 s. The 0.05 setting is a measured 6.4–8.3× headroom, no longer a guess; `bias_tau` 1000 s and `gyro_bias_rw_sigma` 0.002 are consistent. §3.3 |
| **Positive pitch = nose DOWN** | opposite of aerospace convention (measured 2026-08-04). Confirm signs before touching the pitch PID or AUTO scenario. §2.1 |
| Roll/pitch have no offset calibration | **Roll** shows a stable +6.4° over a 3.87-day bench record — that looks like a genuine mounting offset. **Pitch does not**: the "≈ 7.9°" from 2026-08-04 (§2.1) did not reproduce — the same 3.87 days averaged **−0.45°** (range −1.03 … +0.38) and another day read +5.83°. Resting pitch depends on how the robot is set down, so **do not treat 7.9° as a constant**. `hydro_estimator_node` used it as an attitude fallback until 2026-08-25; that fallback is gone (speed is NaN without attitude). |
| Post-assembly tests pending | tail-beat pitch bias, PID sign bench check **before water**. §8.2 (multi-attitude tilt PASSED 2026-08-27: all six poses ≤ 0.61° vs gravity, incl. pitch ±89°, §11.2) |
| `pressure_lag_ms` 72 ms is stale | Sized for the 11 Hz state machine; at 19.3 Hz most samples lag ≈32 ms (≈52 ms on temperature cycles). Harmless today (periodic tail-beat averages to zero and the τ=1 s LPF damps 2 Hz by 12.6×), so `docs/todo.md` gates it behind measuring the tail-beat pitch amplitude first — but start any re-fit from 32, not 72. |
| `data_logger_node` total log volume unbounded | ~84 MB/hour since raw pressure columns (2026-08-21; ~80 MB since 2026-08-18), about +10 MB/hour more since the 8 wet-zero columns and +~15 MB/hour since the 7 MPU9250 columns (both 2026-09-16). Per-file 200 MB rotation exists, but old files are never deleted (deliberate — `log_csv/` also holds the experiment CSVs; the calibration files now live in `config/`). Disk space is managed by hand. |

## Language

Code comments, log messages, and commit history are in **Korean**. Match that when editing — do not
convert existing comments to English.
