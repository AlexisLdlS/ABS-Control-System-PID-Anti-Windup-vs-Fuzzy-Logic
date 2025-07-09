# ABS-Control-System-PID-vs-Fuzzy-Logic
This repository presents two versions of an Anti-lock Braking System (ABS) controller:

  -PID-based ABS Controller: using anti-windup digital PID for slip regulation.
  
  -Fuzzy Logic ABS Controller: using Mamdani fuzzy inference and a yaw correction module.
  
Both systems were tested under realistic driving conditions, including emergency braking at high speeds and surface changes (e.g. split-μ scenarios). The controllers regulate brake pressure based on real-time sensor signals (wheel speed, master cylinder pressure, IMU).
