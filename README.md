# 两轮自平衡小车 🚗

> 基于 STM32F103C8T6 的自平衡小车系统，包含平衡车主控与无线遥控手柄两端，裸机开发，三环串级 PID + 互补滤波，支持蓝牙无线调参。

[![MCU](https://img.shields.io/badge/MCU-STM32F103C8T6-blue)](https://www.st.com)
[![Framework](https://img.shields.io/badge/Library-STD_Library-orange)](#)
[![License](https://img.shields.io/badge/License-MIT-green)](./LICENSE)

---

## 📖 项目简介

本项目是一个完整的自平衡小车系统，由 **平衡车** 和 **遥控器** 两个 STM32F103C8T6 构成，通过 2.4GHz NRF24L01 无线通信，支持：

- 直立平衡 + 遥控行走
- 蓝牙 APP 在线调参（全部 PID 参数）
- OLED 实时显示姿态、参数、PWM 占空比
- 软件 I2C / SPI 纯手写驱动（不依赖硬件外设）

## 🧱 硬件总览

| 部件 | 主控 | IMU | 电机 | 编码器 | 无线 | 显示 | 电源 |
|------|------|-----|------|--------|------|------|------|
| **平衡车** | STM32F103C8T6 | MPU6050 | 直流电机×2 + TB6612 | 正交编码器×2 | NRF24L01 (RX) | OLED 128×64 | 7.4V 锂电池 |
| **遥控器** | STM32F103C8T6 | — | — | — | NRF24L01 (TX) | OLED 128×64 | 3.7V 锂电池 |

## 🎛️ 控制架构

### 三环串级 PID

```
角度环（外环, 100Hz）──→ 速度环（内环, 20Hz）──→ 电机 PWM
         │
   转向环（并联, 差速控制）
```

### PID 参数

| 控制环 | Kp | Ki | Kd | 频率 |
|--------|----|----|-----|------|
| 角度环 | 8.04 | 0.10 | 4.91 | 1kHz |
| 速度环 | 1.12 | 0.12 | — | 200Hz |
| 转向环 | 4.00 | 1.00 | — | 200Hz |

### PID 优化策略

- **微分先行**：对测量值微分，避免目标突变冲击
- **积分分离**：大偏差关积分，小偏差开积分 → 减小超调
- **积分限幅 + 输出限幅**：防止积分饱和与 PWM 溢出
- **死区补偿**（OutOffset）：消除电机死区

> ⚠️ 角度超过 ±50° 自动停机保护。

## 🧮 姿态解算

一阶互补滤波，融合加速度计与陀螺仪：

```
倾角 = 0.02 × atan2(AY, AZ) + 0.98 × (Angle + Gyro·dt)
```

- 加速度计权重：2%（低频可靠，高频噪声大）
- 陀螺仪权重：98%（高频准确，低频漂移）

### 系统时序

基于 TIM1 中断（1kHz）分频调度：

| 频率 | 执行内容 |
|------|----------|
| 1kHz | MPU6050 读取 + 姿态解算 + 角度环 PID + PWM 更新 |
| 200Hz | 编码器速度计算 + 速度环 PID + 转向环 PID |

## 📡 通信协议

### 无线通信（NRF24L01）

| 参数 | 值 |
|------|-----|
| 速率 | 2Mbps |
| 地址 | 5 字节 |
| 载荷 | 32 字节 |
| 模式 | 自动应答 + 自动重传 |
| 帧格式 | `[ID, LH, LV, RH, RV, KEY]` |

### 蓝牙调参（HC-05 / BT04）

- 接口：USART2，9600bps
- 功能：APP 实时修改 Kp / Ki / Kd 全部参数

## 📂 工程结构

```
SelfBalancingCar-STM32/
├── README.md                    # 本文件
├── .gitignore
├── 原理图/                      # 嘉立创 EDA 原理图
│   ├── ProPrj_SCH_华思艺平衡小车.epro2
│   └── ProPrj_华思艺遥控手柄.epro2
│
├── 平衡车代码/                  # 平衡车主控工程
│   ├── Hardware/                # 外设驱动层
│   │   ├── MPU6050.c/h          # 6 轴 IMU 驱动（软件 I2C）
│   │   ├── Encoder.c/h          # 正交编码器（TIM3/4 编码器模式）
│   │   ├── Motor.c/h            # TB6612 电机驱动
│   │   ├── PWM.c/h              # PWM 输出
│   │   ├── NRF24L01.c/h         # 2.4GHz 无线接收
│   │   ├── BlueSerial.c/h       # 蓝牙串口
│   │   ├── OLED.c/h             # OLED 128×64 显示
│   │   ├── MyI2C.c/h            # 软件 I2C
│   │   ├── Serial.c/h           # 串口驱动
│   │   ├── Key.c/h              # 按键
│   │   └── LED.c/h              # LED 指示
│   ├── System/                  # 系统层
│   │   ├── Delay.c/h            # 微秒/毫秒延时
│   │   └── Timer.c/h            # TIM1 中断调度
│   ├── User/                    # 应用层
│   │   ├── main.c               # 主程序入口
│   │   ├── PID.c/h              # 三环串级 PID 核心
│   │   └── stm32f10x_it.c/h     # 中断服务
│   ├── Library/                 # STM32F10x 标准外设库
│   ├── Start/                   # CMSIS 启动文件
│   ├── Project.uvprojx          # Keil 工程
│   └── README.md
│
└── 遥控器代码/                  # 遥控手柄工程
    ├── Hardware/                # 外设驱动层
    │   ├── AD.c/h               # ADC 摇杆采集
    │   ├── NRF24L01.c/h         # 2.4GHz 无线发送
    │   ├── OLED.c/h             # OLED 128×64 显示
    │   └── Key.c/h              # 按键扫描
    ├── System/                  # 系统层
    │   ├── Delay.c/h            # 微秒/毫秒延时
    │   └── Timer.c/h            # 定时器
    ├── User/                    # 应用层
    │   ├── main.c               # 主程序
    │   └── stm32f10x_it.c/h     # 中断服务
    ├── Library/                 # STM32F10x 标准外设库
    ├── Start/                   # CMSIS 启动文件
    ├── Project.uvprojx          # Keil 工程
    └── README.md
```

## 🔧 开发环境

| 工具 | 用途 |
|------|------|
| **Keil MDK-ARM** | 编译、下载、调试 |
| **STM32 标准外设库** | 底层驱动（非 HAL） |
| **VSCode** | 代码编辑 |
| **嘉立创 EDA** | 原理图 & PCB 设计 |
| **蓝牙调试 APP** | PID 参数在线调整 |

## 🚀 快速开始

1. 用 Keil MDK-ARM 打开 `平衡车代码/Project.uvprojx`
2. 编译下载到平衡车主控板
3. 打开 `遥控器代码/Project.uvprojx`，编译下载到遥控手柄
4. 上电后小车自动直立平衡，用遥控器摇杆控制行走
5. 通过蓝牙 APP 连接 HC-05，可实时调整 PID 参数

## ✨ 功能清单

- [x] 直立平衡 + 遥控行走
- [x] 三环串级 PID 控制（角度环 + 速度环 + 转向环）
- [x] 蓝牙 APP 在线修改全部 PID 参数
- [x] OLED 实时显示角度、参数、占空比
- [x] 软件 I2C / SPI 纯手写驱动
- [x] 角度超限自动停机保护
- [x] NRF24L01 2.4GHz 无线通信
- [x] EasyEDA 原理图

---

> 开发周期：2023.10 – 2024.01
