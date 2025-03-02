# 🎮 PS4-Controlled Holonomic Robot with IMU Stabilization

## 🌟 Overview

This project implements a 4-wheeled holonomic robot controlled by a PS4 controller, featuring IMU-based orientation stabilization using the BNO055 sensor. Built on the STM32F407 platform, the robot offers precise omnidirectional movement capabilities with real-time orientation correction.

## ⚡ Features

- **Omnidirectional Movement**: Full holonomic control using mecanum/omni wheels
- **Wireless Control**: PS4 controller integration via I2C communication
- **Orientation Stabilization**: BNO055 IMU-based heading correction
- **Real-time Feedback**: Serial monitoring of movement parameters and sensor data
- **High-Performance**: Runs at 168MHz for responsive control

## 🔧 Hardware Requirements

- STM32F407 Development Board
- BNO055 IMU Sensor
- PS4 Controller
- 4x DC Motors with Encoders
- 4x Motor Drivers
- Power Supply (12V recommended)
- Mecanum/Omni Wheels

## 📡 Pin Configuration

### I2C1 (BNO055 IMU Communication)

- SCL: PB8
- SDA: PB9

### I2C2 (PS4 Controller Communication)

- SCL: PB10
- SDA: PB11

### PWM Outputs (Motor Control)

- Motor 1: PD12 (TIM4_CH1)
- Motor 2: PD13 (TIM4_CH2)
- Motor 3: PD14 (TIM4_CH3)
- Motor 4: PD15 (TIM4_CH4)

## 💻 Software Dependencies

- STM32 HAL Library
- Custom BNO055 Driver
- Custom I2C Communication Protocol
- STM32CubeIDE/Keil Development Environment

## 🚀 Setup Instructions

1. Clone this repository
2. Open the project in your STM32 IDE
3. Configure your toolchain and debugger
4. Build and flash to your STM32F407 board
5. Connect the hardware according to the pin configuration
6. Power up and pair your PS4 controller

## 🎯 Usage

1. Power on the robot and ensure the PS4 controller is paired
2. Use the left joystick for translational movement
3. Use the right joystick for rotational movement
4. The robot will automatically maintain its heading using IMU feedback

## 🔄 Control Logic & Kinematics

### Holonomic Drive System

The robot implements a four-wheeled holonomic drive system using mecanum wheels. The kinematics model accounts for both translational and rotational velocities in a planar coordinate system.

#### Velocity Decomposition

The system decomposes user input into three primary velocity components:

- \( v_x \): X-axis translational velocity
- \( v_y \): Y-axis translational velocity
- \( v_w \): Angular velocity (rotation)

The velocity components are calculated using:

```c
v = sqrt((pow(x, 2) + pow(y, 2)));  // Magnitude of translational velocity
angle = atan2(y, x);                // Direction of translation
v_x = v * cos(angle);               // X component
v_y = v * sin(angle);               // Y component
```

#### Inverse Kinematics

The wheel velocities are computed using the following inverse kinematics equations:

```c
w1 = (0.70711 * (-v_x + v_y)) + w + correction;  // Front Left
w2 = (0.70711 * (-v_x - v_y)) + w + correction;  // Front Right
w3 = (0.70711 * (v_x - v_y)) + w + correction;   // Rear Left
w4 = (0.70711 * (v_x + v_y)) + w + correction;   // Rear Right
```

Where:

- 0.70711 is approximately \( \frac{1}{\sqrt{2}} \)
- `correction` is the IMU-based orientation correction term

#### Orientation Control

The system implements a PID-based orientation control system:

```c
error = curr_angle - targated_angle;
// Angle wrapping for shortest path rotation
if (error < -180) {
    error = error + 360;
}
else if (error > 180) {
    error = error - 360;
}
diff = error - prev_error;
correction = (error * KP) + (diff * KD);
```

## 💻 Bare-Metal Implementation Details

### Clock Configuration

The system runs at maximum frequency (168MHz) with the following configuration:

#### PLL Settings

```c
osc_init.PLL.PLLM = 8;
osc_init.PLL.PLLN = 336;
osc_init.PLL.PLLP = 2;
osc_init.PLL.PLLQ = 2;
```

This yields:

- System Clock (SYSCLK): 168MHz
- AHB Clock: 168MHz
- APB1 Clock: 42MHz
- APB2 Clock: 84MHz

### Timer Configuration

#### PWM Generation (TIM4)

- Clock Source: APB1 (42MHz)
- Period: 17500 - 1
- Prescaler: 4999
- PWM Frequency: ~39kHz
- Resolution: 17500 steps

Channels are configured for PWM1 mode with varying duty cycles:

```c
TIM4_CH1 (PD12): Motor 1 Control
TIM4_CH2 (PD13): Motor 2 Control
TIM4_CH3 (PD14): Motor 3 Control
TIM4_CH4 (PD15): Motor 4 Control
```

### I2C Communication

#### I2C1 Configuration for BNO055 IMU

- Mode: Standard Mode (SM)
- Clock Speed: 100kHz
- Addressing Mode: 7-bit
- Duty Cycle: 2 (FM mode)
- GPIO Configuration:
  ```c
  SCL (PB8): Alternate Function, Open-Drain
  SDA (PB9): Alternate Function, Open-Drain
  Pull-up resistors enabled
  ```

#### I2C2 Configuration for PS4 Controller

- Mode: Standard Mode (SM)
- Clock Speed: 100kHz
- Addressing Mode: 7-bit
- Duty Cycle: 2 (FM mode)
- GPIO Configuration:
  ```c
  SCL (PB10): Alternate Function, Open-Drain
  SDA (PB11): Alternate Function, Open-Drain
  Pull-up resistors enabled
  ```

#### Data Processing

PS4 controller data is processed in 5-byte packets:

```c
rcv_buf[0]: Left Stick X-axis  (0-255)
rcv_buf[1]: Left Stick Y-axis  (0-255)
rcv_buf[2]: Right Stick X-axis (0-255)
rcv_buf[3]: Button State 1
rcv_buf[4]: Button State 2
```

### BNO055 IMU Integration

#### I2C1 Configuration

- Standard Mode Operation
- Hardware Address: 0x28
- Operating Mode: NDOF (Nine Degrees of Freedom)
- Update Rate: ~100Hz
- GPIO Configuration:
  ```c
  SCL (PB8): Alternate Function, Open-Drain
  SDA (PB9): Alternate Function, Open-Drain
  Pull-up resistors enabled
  ```

#### Orientation Data Processing

```c
bno055_vector_t BNO_OP = bno055_getVectorEuler();
curr_angle = BNO_OP.x;
if (curr_angle) {
    curr_angle = curr_angle - 180.0;
}
```

### GPIO Configuration

#### Timer GPIO (GPIOD)

```c
Mode: Alternate Function Push-Pull
Speed: Fast (50MHz)
Alternate Function: AF2 (TIM4)
Pins: PD12, PD13, PD14, PD15
```

#### I2C GPIO (GPIOB)

```c
Mode: Alternate Function Open-Drain
Speed: Fast (50MHz)
Pull-up: Enabled
Alternate Function: AF4 (I2C)
Pins: PB10 (SCL), PB11 (SDA)
```

### Error Handling and System Protection

- Flash latency configured based on clock frequency
- Voltage scaling set to Scale 1 for high-frequency operation
- Systick configured for 1ms time base
- Hardware initialization verification
- I2C communication error detection
- Motor PWM safety limits

### Memory Management

#### Stack Configuration

- Main Stack Size: 0x400 bytes
- Process Stack Size: 0x200 bytes
- Heap Size: 0x200 bytes

#### Memory Map

- Flash: 0x08000000 - 0x080FFFFF (1MB)
- SRAM: 0x20000000 - 0x2001FFFF (128KB)
- CCM RAM: 0x10000000 - 0x1000FFFF (64KB)

### Optimization Techniques

1. **Critical Section Management**

   ```c
   __disable_irq();  // Enter critical section
   // Critical code
   __enable_irq();   // Exit critical section
   ```

2. **DMA Usage**

   - Direct memory access for I2C communication
   - Reduces CPU overhead during data transfer

3. **Interrupt Priority**
   - SysTick: Priority 15 (lowest)
   - I2C: Priority 10
   - Timer: Priority 5

### Performance Metrics

- Control Loop Frequency: 1kHz
- PWM Resolution: 17500 steps
- I2C Data Rate: 100kbps
- Orientation Update Rate: 100Hz
- System Response Time: <5ms

## 📊 System Architecture

```
[PS4 Controller] --I2C2--> [STM32F407] <--I2C1-- [BNO055 IMU]
         |
         v
    Processing
         |
         v
    [Motor Control]
         |
         v
[Omnidirectional Wheels (4x)]
```

## 🛠️ Advanced Features

- PID-based orientation correction
- Velocity mapping and scaling
- Real-time telemetry output
- Configurable control parameters

## 📊 Performance

- Control loop frequency: 1kHz
- PWM frequency: ~39kHz (with prescaler 4999)
- System clock: 168MHz
- I2C communication speed: Standard Mode

## 🤝 Contributing

Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## 📝 License

[MIT License](LICENSE)

## 🔍 Troubleshooting

Common issues and their solutions:

1. **Controller not connecting**: Verify I2C address and communication parameters
2. **Erratic movement**: Check motor connections and PWM configuration
3. **IMU errors**: Ensure proper BNO055 initialization and calibration

## 📞 Contact

For questions and support, please open an issue in the GitHub repository.

## 🌟 Acknowledgments

- STM32 Community
- BNO055 Driver Contributors
- PS4 Controller Interface Developers
