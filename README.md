# 🎛️ DMX Master STM32

## 📌 Overview

The **DMX-Master-STM32** is part of an integrated system that includes a **graphical interface for Windows**, developed in **PyQt**. This interface allows users to send DMX commands, interact with RDM devices, and visualize DMX/RDM frames in real time. Communication between the software and the board occurs via **USB-C**, enabling control and monitoring of connected devices.

🔗 **Link to the graphical interface project:** [GUI_RDM_DMX_Master](https://github.com/GuilhermeRS11/GUI_RDM_DMX_Master/tree/master)

In addition to this software integration, the board also features its **own embedded graphical interface**, displayed on a **0.96'' OLED screen** and controlled via **four physical buttons**.

💡 **Project Status:**  
✅ **Hardware ready** (PCB designed with **Altium Designer 25.1.2**).  
⚙️ **Firmware and embedded interface under development** (**STM32Cube Version: 1.14.1**).  

---

## 🖥️ Embedded Graphical Interface  

The onboard graphical interface allows users to configure and control **DMX and RDM** devices directly, without requiring external software.  

🛠 **Features of the embedded interface:**  
- 🎚️ Adjustment of **DMX and RDM** parameters.  
- 📍 **Device addressing** configuration.  
- 📡 **Monitoring** of received information.  
- 🔘 Direct control via physical buttons.  

Thus, if the Windows graphical software is unavailable, the board can be powered by **5V** and used independently.

---

## ⚙️ About the DMX-Master-STM32 Project

The **DMX-Master-STM32** is responsible for receiving data via **USB-C**, interpreting it, and converting it to the **DMX standard**, enabling communication with lighting and control devices. It can operate either when connected to the PC GUI or autonomously.

### 🛠️ PCB Features:
✅ **USB-C connectivity** for communication with external software.  
✅ **5V → 3.3V voltage regulator** for safe circuit operation.  
✅ **CH340G USB converter** for serial interface.  
✅ **STM32C031K6T6 microcontroller** for processing and control.  
✅ **MAX485 converter** for RS485 interface, compatible with DMX.  
✅ **4 physical buttons** for direct interaction with the board.  
✅ **128x64 OLED display (0.96'')** with menus and manual adjustments.  

---

## 📡 Schematic Visualization  

Here are some images of the project's schematic circuit:  

![📜 Schematic - Page 1](documents/images/schematic_1.png)  
![📜 Schematic - Page 2](documents/images/schematic_2.png)  

---

## 🔧 Hardware Visualization  

Here are some images of the PCB's 3D model:  

![🖼️ DMX-Master-STM32 - Front View](documents/images/3D_PCB_front_view.png)  
![🖼️ DMX-Master-STM32 - Back View](documents/images/3D_PCB_back_view.png)  

---

## 📜 License

This project is licensed under **[GPLv3](https://www.gnu.org/licenses/gpl-3.0.txt)**, allowing use, modification, and distribution of the code as long as the same freedoms are maintained.

---