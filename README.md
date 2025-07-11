# ABS-Control-System-PID-Anti-Windup-vs-Fuzzy-Logic
This repository presents two versions of an Anti-lock Braking System (ABS) controller:

  -PID-based ABS Controller: using anti-windup digital PID for slip regulation.

<img width="1044" height="531" alt="PID anti windup Diagram" src="https://github.com/user-attachments/assets/58037775-e892-4b31-b714-9a57ad5619bc" />

  -Fuzzy Logic ABS Controller: using Mamdani fuzzy inference and a yaw correction module.

<img width="650" height="364" alt="Fuzzy Controller Diagram" src="https://github.com/user-attachments/assets/c8e10027-ee7d-4af7-9036-3e6c4f62865f" />
  
Each controller was implemented within a code-based Simulink block, mimicking the application-layer logic that would typically run on an ECU.

Both systems were tested under realistic driving conditions, including emergency braking at high speeds and surface changes (e.g. split-μ scenarios). The controllers regulate brake pressure based on real-time sensor signals (wheel speed, master cylinder pressure, IMU). 

- System Model:

![image](https://github.com/user-attachments/assets/232b91fa-71d2-4884-a375-d94c8198d428)

- Brake Controller Block:

![image](https://github.com/user-attachments/assets/cc6d4fab-8c94-4239-a2f3-d6134da129dd)

- 3D Simulation Movie Engine Available

<img width="1131" height="887" alt="3d Simulation Move" src="https://github.com/user-attachments/assets/46d4b7fb-72b4-418a-9ef1-e04e87c65403" />


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
- Selection of the slip target based on the max. adhesion point in the friction-slip curve.
- PID controller tuning using test sweeps of \( K_p, K_i, K_d \)
- Fuzzy membership functions (triangular) and Mamdani inference
- Custom yaw correction by asymmetric brake force distribution
- Anti-windup strategy to prevent actuator saturation

<img width="707" height="479" alt="Wheel Friction Slip curve" src="https://github.com/user-attachments/assets/9e395f89-23c5-4a39-ae54-e7a36fca266a" />

<img width="475" height="458" alt="Wheel and brake Slip Diagram Formula" src="https://github.com/user-attachments/assets/25643caf-e1b0-4870-94d4-8dbeaf9e4e47" />

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
For 
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

---

## NOTES
For running and simulating the model: 
- Following ADD-On needs to be instaled in Simulink: Vehicle Dynamics Blockset by MathWorks

Steps for executing the simulation:

1.- Open the ABS_Project Folder

2.- Run one of the INITIAL CONFIG SCRIPTs (Init_*.m)

  Script will define:

  - Sampling period of the inputs to simulation and the controller.
  - Multiple model parameters.
  - Surface definition. Coefficients of fricition for the wheels.
  - The maneuver to be carried out by a ficticious driver, based on throttle, brake and steering inputs.

3.- Open the Main System Simulink Model File (Main_A.slx)

4.- Enable/Disable 3D Simulation Movie: go to Visualization block and double click the "3D Engine" block, selecting the corresponding option and return to Main Model Block.

5.- Run the simulation by pressing the Run Button (F5)

---

## LICENSE
This project is for demonstration and educational purposes.  All rights reserved unless otherwise stated. Not to be used or redistributed without author credit. 
