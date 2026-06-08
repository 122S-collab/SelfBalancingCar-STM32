# 两轮自平衡小车 🚗

> 基于 STM32F103C8T6 的自平衡小车系统，含平衡车主控与无线遥控手柄两端，裸机开发，三环串级 PID + 互补滤波，支持蓝牙无线调参。

[![MCU](https://img.shields.io/badge/MCU-STM32F103C8T6-blue)](https://www.st.com)
[![Framework](https://img.shields.io/badge/Library-STD_Library-orange)](#)
[![License](https://img.shields.io/badge/License-MIT-green)](./LICENSE)

---

## 📖 项目简介

本项目由 **平衡车** 和 **遥控器** 两个 STM32F103C8T6 构成，通过 2.4GHz NRF24L01 无线通信。

- 直立平衡 + 遥控行走
- 蓝牙 APP 在线调参（全部 PID 参数）
- OLED 实时显示姿态、参数、PWM 占空比

## 🧱 硬件总览

| 部件 | 主控 | IMU | 电机 | 编码器 | 无线 | 显示 | 电源 |
|------|------|-----|------|--------|------|------|------|
| **平衡车** | STM32F103C8T6 | MPU6050 | 直流电机×2 + TB6612 | 正交编码器×2 | NRF24L01 (RX) | OLED 128×64 | 7.4V 锂电池 |
| **遥控器** | STM32F103C8T6 | — | — | — | NRF24L01 (TX) | OLED 128×64 | 3.7V 锂电池 |

## 🎛️ 控制架构

三环串级 PID：**角度环**（外环, 1kHz）→ **速度环**（内环, 200Hz）→ 电机 PWM，**转向环**并联差速控制。

| 控制环 | Kp | Ki | Kd | 要点 |
|--------|----|----|-----|------|
| 角度环 | 8.04 | 0.10 | 4.91 | 微分先行，积分分离 |
| 速度环 | 1.12 | 0.12 | — | 积分限幅 + 输出限幅 |
| 转向环 | 4.00 | 1.00 | — | 死区补偿（OutOffset） |

> ⚠️ 角度超过 ±50° 自动停机。

## 🧮 姿态解算

```
倾角 = 0.02 × atan2(AY, AZ) + 0.98 × (Angle + Gyro·dt)
```

一阶互补滤波：加速度权 2%，陀螺仪权 98%。基于 TIM1 1kHz 中断调度，200Hz 分频处理速度环。

## 📡 通信

- **NRF24L01**：2Mbps，5 字节地址，32 字节载荷，自动应答 + 重传，帧格式 `[ID, LH, LV, RH, RV, KEY]`
- **蓝牙 HC-05**：USART2，9600bps，APP 实时修改全部 Kp/Ki/Kd

## 📂 工程结构

```
SelfBalancingCar-STM32/
├── README.md
├── .gitignore
├── 原理图/                              # 嘉立创 EDA 工程 (.epro2)
│   ├── ProPrj_SCH_华思艺平衡小车.epro2
│   └── ProPrj_华思艺遥控手柄.epro2
│
├── 平衡车代码/                           # 平衡车主控（Keil MDK + 标准外设库）
│   ├── Hardware/                         # 手写外设驱动
│   │   ├── MPU6050.c/h                   # IMU 驱动 — 软件 I2C 协议
│   │   ├── Encoder.c/h                   # 正交编码器 — TIM3/4 编码器模式
│   │   ├── Motor.c/h + PWM.c/h           # TB6612 H 桥 + PWM 调速
│   │   ├── NRF24L01.c/h                  # 2.4GHz 接收 — 软件 SPI 协议
│   │   ├── BlueSerial.c/h                # HC-05 蓝牙透传
│   │   ├── OLED.c/h + OLED_Data.c/h      # 128×64 I2C 显示屏
│   │   ├── MyI2C.c/h                     # 软件 I2C 总线实现
│   │   ├── Serial.c/h                    # USART 串口
│   │   └── Key.c/h + LED.c/h             # 按键输入 + LED 指示
│   ├── System/                           # 系统层
│   │   ├── Delay.c/h                     # 微秒/毫秒延时
│   │   └── Timer.c/h                     # TIM1 中断 + 时序分频调度
│   ├── User/                             # 应用层
│   │   ├── main.c                        # 主循环：初始化 → 姿态解算 → PID → 通信
│   │   ├── PID.c/h                       # 三环串级 PID 完整实现
│   │   └── stm32f10x_it.c/h              # 中断服务函数
│   ├── Library/                          # STM32F10x 标准外设库（非 HAL）
│   ├── Start/                            # CMSIS 启动文件 + 系统时钟配置
│   └── Project.uvprojx                   # Keil MDK 工程文件
│
└── 遥控器代码/                           # 遥控手柄（Keil MDK + 标准外设库）
    ├── Hardware/                         # 手写外设驱动
    │   ├── AD.c/h                        # 双摇杆 ADC 四通道采集
    │   ├── NRF24L01.c/h                  # 2.4GHz 发送 — 软件 SPI 协议
    │   ├── OLED.c/h + OLED_Data.c/h      # 128×64 I2C 显示屏
    │   └── Key.c/h                       # 轻触按键扫描
    ├── System/                           # Delay + Timer 系统层
    ├── User/                             # main.c + 中断服务 + 业务逻辑
    ├── Library/                          # STM32F10x 标准外设库
    ├── Start/                            # CMSIS 启动文件
    └── Project.uvprojx                   # Keil MDK 工程文件
```

## 🔧 开发环境

| 工具 | 用途 |
|------|------|
| **Keil MDK-ARM** | 编译 / 下载 / 调试 |
| **STM32 标准外设库** | 底层寄存器驱动（非 HAL） |
| **VSCode** | 代码编辑 |
| **嘉立创 EDA** | 原理图 & PCB 设计 |
| **蓝牙调试 APP** | 无线 PID 参数整定 |

## 🚀 快速开始

1. 用 Keil MDK-ARM 打开 `平衡车代码/Project.uvprojx`，编译下载到主控板
2. 打开 `遥控器代码/Project.uvprojx`，编译下载到遥控手柄
3. 小车平放上电 → MPU6050 自动校准（保持静止）→ 自动进入直立平衡
4. 遥控器摇杆控制前后行走 / 原地转向
5. 手机蓝牙连接 HC-05（9600bps），用调试 APP 实时调整 PID 参数

## ✨ 功能清单

- [x] 直立平衡 + 无线遥控行走
- [x] 三环串级 PID（角度环 + 速度环 + 转向环）
- [x] 蓝牙 APP 在线修改全部 PID 参数（Kp/Ki/Kd）
- [x] OLED 实时显示角度、参数、PWM 占空比
- [x] 软件 I2C / SPI 手写驱动
- [x] 角度超限（±50°）自动停机保护
- [x] NRF24L01 2.4GHz 无线通信
- [x] 嘉立创 EDA 原理图设计

---

> 开发周期：2023.10 – 2024.01
