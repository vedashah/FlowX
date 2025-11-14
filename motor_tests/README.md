# 🧪 Motor Tests  
### *FlowX Cold-Gas Thruster Test Bench – Motor & Actuation Validation Suite*

The `motor_tests` directory contains all scripts, utilities, and documentation related to validating motor-driven systems used within the FlowX platform.  
Although FlowX uses cold-gas thrusters for primary actuation, several auxiliary systems (e.g., test-stand mechanisms, valve actuators, safety shutters, and calibration rigs) rely on electric motors.  
This folder provides the full testing environment for those components.

---

## 🎯 Purpose

The goal of `motor_tests` is to:

- Verify proper motor actuation and response behavior  
- Test PWM output, ESC communication, and motor speed stability  
- Validate load response under simulated thrust cycles  
- Characterize startup latency, overshoot, and rotational stability  
- Provide reproducible scripts for integration testing with the Raspberry Pi  

This suite is essential for ensuring the reliability of mechanical subsystems supporting the cold-gas propulsion test bench.

---

## 📂 Folder Contents

`motor_tests/` contains:

- **test_pwm.py** – PWM signal generation and validation  
- **test_esc.py** – Initialization and ramp tests for ESC-driven motors  
- **test_load.py** – Load response and torque variation testing  
- **motor_controller.py** – Unified motor control interface for reuse  
- **README.md** – Overview and instructions for this module  
- **/logs** – Auto-generated test logs, timing data, and response plots (ignored by Git)  

*(Note: File names above are placeholders — you can replace them with your actual script names.)*

---

## ⚙️ Setup Requirements

Before running tests:

1. Ensure the Raspberry Pi GPIO is accessible over SSH  
2. Install required libraries:  

   ```bash
   sudo apt install python3-gpiozero python3-rpi.gpio -y
   pip3 install pigpio
