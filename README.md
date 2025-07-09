# ABS-Control-System-PID-Anti-Windup-vs-Fuzzy-Logic
This repository presents two versions of an Anti-lock Braking System (ABS) controller:

  -PID-based ABS Controller: using anti-windup digital PID for slip regulation.
  
  -Fuzzy Logic ABS Controller: using Mamdani fuzzy inference and a yaw correction module.
  
Both systems were tested under realistic driving conditions, including emergency braking at high speeds and surface changes (e.g. split-μ scenarios). The controllers regulate brake pressure based on real-time sensor signals (wheel speed, master cylinder pressure, IMU).



## 🚗 Control Strategy Overview

| Feature                            | PID Controller                  | Fuzzy Logic Controller             |
|-----------------------------------|----------------------------------|------------------------------------|
| Control Approach                  | Digital PID + Anti-windup        | Mamdani-type fuzzy inference       |
| Input Signals                     | Slip error per wheel             | Slip error per wheel               |
| Output                            | Brake pressure per caliper       | Brake pressure per caliper         |
| Adaptive to surface differences   | Manual tuning                    | Rule-based dynamic adjustment      |
| Yaw Control                       | Not included                     | Custom yaw mitigation block        |
| Simulated Scenarios               | Panic brake, light brake, split-μ | Split-μ with yaw imbalance         |
| Max Braking Distance Reduction    | **53%**                          | **35%** + improved yaw handling    |
| Tools                             | MATLAB/Simulink                  | MATLAB/Simulink                    |

---

## 🔬 Key Technical Concepts

- Slip estimation from wheel angular speed and IMU (via Riemann integration)
- PID controller tuning using test sweeps of \( K_p, K_i, K_d \)
- Fuzzy membership functions (triangular) and Mamdani inference
- Custom yaw correction by asymmetric brake force distribution
- Anti-windup strategy to prevent actuator saturation

---

## 📈 Performance Highlights

### PID Controller
- Up to 53% braking distance reduction in emergency stops (Init Panic90)
- Controlled slip converges to 0.2, with minor oscillations
- Braking stability sensitive to coefficient tuning

### Fuzzy Logic Controller
- 35% braking distance reduction in split-μ (Init Split70)
- Reduced yaw deviation; vehicle remains in lane
- Smooth pressure modulation across asymmetric friction

---

## 🛠️ Future Improvements

- Integrate a model predictive control (MPC) or neural-based approach
- Enhance slip convergence at low speeds
- Fully adaptive yaw control (e.g., fuzzy or PID-based)
- Consider additional inputs: drivetrain layout, aero drag, weight distribution

---

## 🧠 My Technical Contribution
I was directly responsible for:
- Designing and coding the ABS control architecture and strategies (PID and fuzzy logic)
- Coding the slip estimation, braking logic, and yaw correction mechanisms
- Implementing MATLAB/Simulink models
- Supported the Tuning of control parameters and validating performance through simulations
