## 🔧 Hardware Used

- **SparkFun RedBoard** (Arduino Uno compatible)  
- **IR Sensor Module**  
  - Digital output: `1 = light detected`, `0 = blocked`  
- **DC Motor (Fan)**  
- **Motor Driver** (e.g., transistor or motor driver IC)  
- **External Power Supply** (recommended for motor stability)  

---

## 🧵 System Architecture

The system runs four concurrent FreeRTOS tasks:

- **Measurement Task** – Calculates RPM using interrupt-based IR pulse counting  
- **Speed Control Task** – Adjusts PWM output to reach target speed  
- **Input Task** – Reads user input (Serial commands)  
- **Display Task** – Outputs system data (RPM, setpoint, PWM)  
