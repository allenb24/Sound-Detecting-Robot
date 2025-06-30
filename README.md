# Sound Detecting Robot with Dual Microphones (MSP432)

This embedded systems project uses two microphones and an MSP432P401R microcontroller to control a robot based on the direction of detected sound. The robot can determine whether sound is stronger on the left or right and react by moving forward, turning, stopping, or rotating accordingly. The project included creating a circuit to embed with the microcontroller and program in C.

Developed as a final project for EEC 10: Introduction to Embedded Systems at UC Davis.

---

## Features

- **Dual Microphone Sampling**:
  - Captures analog signals via ADC14 on pins P6.0 and P6.1.
  - Real-time sampling and filtering of sound data.

- **Digital Filtering**:
  - Implements a two-stage filter: high-pass followed by low-pass.
  - Reduces ambient noise and highlights sudden sound bursts.

- **Direction Detection Logic**:
  - Divides sound input into chunks, extracts average maximums.
  - Compares left vs. right channel intensity to decide robot behavior.

- **Motor Control with PWM**:
  - Uses Timer A0 to drive left/right wheels via PWM on P2.6/P2.7.
  - Supports multiple directional states: Forward, Left, Right, Stop, Rotate.

- **Timer Interrupt-Based Execution**:
  - Timer A2 interrupt handles sound sampling and processing.
  - Uses SysTick timer for delay-based motor control.

---

## Project Structure
- `src/`: Source files, including robot control logic and system initialization
- `inc/`: Header files for system utilities (e.g. Clock, SysTick)
- `ccs_project/`: Code Composer Studio project files
- `media/`: Images and videos of robot

## Robot Design
![robot](https://github.com/user-attachments/assets/9a436956-8c9e-46dd-bac9-63e8cac98ffe)


## Demo


https://github.com/user-attachments/assets/1dd77673-c3ed-451e-aad0-8a63ae4053c5



---

## What I Learned

This project taught me fundamental embedded systems concepts such as:
- Real-time signal processing on microcontrollers
- Motor control using timers and PWM
- Interrupt-driven architecture
- Data filtering and ADC processing
- Debugging hardware interactions (noise, analog signals, GPIO control)
- Circuit design

---

## Educational Use & Acknowledgments

Some files (e.g. `startup_msp432p401r_ccs.c`, `system_msp432p401r.c`) are derived from TI DriverLib and/or provided by UC Davis instructors. These files retain their original license headers.

This repository is shared for **educational and non-commercial use** only.  
Please do **not** reuse this work for academic credit without permission.

It also reinforced how important structured logic and noise filtering are when designing reactive systems. Timing and signal conditioning made a major difference in detecting actual sound events vs. background interference.

---

## License

Educational Use Only – Mixed license due to inclusion of TI and UC Davis starter files. Do not redistribute for commercial or academic credit without proper attribution.
