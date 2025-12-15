
# 🤖 Parallel Robotic Arm

A **2 Degree of Freedom (2-DOF)** robotic arm designed to draw shapes from a predefined set of points.
The system uses **two micro servo motors** for actuation, controlled by an **Arduino Uno** and an **L298N motor driver**.

Each arm segment has a **reach of 78 mm**, with a focus on tidy wiring, compact assembly, and stable mounting of all components.

## 🩺 Context

This project is developed under the **Applied Medical Robotics** module, with inspiration from **robotic-assisted surgical systems** such as:

* Eye dissection
* Colon dissection
* Microvascular / Reconstructive surgery


## ⚙️ Requirements

* **Robust** – stable operation and solid mechanical structure
* **Lightweight** – easy to move and mount
* **Precise** – smooth and accurate positioning
* **Creative** – adaptable to different control and testing scenarios

## 🧩 Components

* Arduino Uno
* L298N Motor Driver
* 2× EMG30 Motors
* Custom 3D-printed or laser-cut arm structure (78 mm reach per arm)

## 📐 Features

* Two-axis motion (2-DOF)
* Controlled shape drawing from coordinate data
* Modular design for experimentation
* Clean and secure wiring setup

## 👩🏻‍💻 Software Architecture 
Python GUI
   ↓  (Serial: "A,B")
Arduino Controller
   ├── Encoder Reading
   ├── Angle Estimation (Virtual Target Generator)
   ├── PID Motor Control
   ├── DC Motors + Encoders
   └── Position Feedback

## 📄 License

This project is licensed under the [MIT License](https://choosealicense.com/licenses/mit/).

This project is part of the 7MRI0060 Applied Medical Robotics module @King's College London


