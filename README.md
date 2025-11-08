# Fixed vs Single axis Rotating Solar Panels — Real-Time Power Comparison

A compact system to compare **fixed vs. rotating solar panels** using **Arduino + Python Dashboard**.

---

## ⚙️ Hardware Overview
| Component | Quantity | Function |
|------------|-----------|-----------|
| Arduino UNO | 1 | Main controller |
| LDR sensors | 2 | Light intensity detection |
| Servo motor | 1 | Panel rotation |
| INA219 sensors | 2 | Voltage, current, and power measurement |
| Solar Panels | 2 | Fixed & rotating comparison |

---

## 📂 Repository Structure

```
SolarTracker-Comparison/
│
├── Arduino/
│ └── SolarTracker_Integrated.ino
│
├── Python/
│ ├── dashboard.py
│ ├── requirements.txt
│
├── images/
│ ├── block_diagram.png
│ ├── circuit_diagram.png
│ └── hardware_setup.png
│
├── docs/
│ └── formulae.md 
│
└── README.md
```

---
<details>
<summary>Arduino Setup</summary>

**📂 Path:** `Arduino/SolarTracker_Integrated.ino`

**Libraries Used:**
- `Servo.h`
- `Wire.h`
- `Adafruit_INA219.h`

**Features:**
 - Reads LDRs and applies **offset calibration**

 ![offset](https://latex.codecogs.com/svg.image?offset%20=%20avg(LDR1)%20-%20avg(LDR2))

- Smooths readings with **Exponential Moving Average (EMA)**

![ema](https://latex.codecogs.com/svg.image?S_t%20=%20\alpha%20D_t%20+%20(1-\alpha)S_{t-1})

-  Calculates power: 

![power](https://latex.codecogs.com/svg.image?P%20=%20V%20\times%20I)

- Sends serial CSV data to Python:

```
LDR1,LDR2,smoothDiff,pos,V_fixed,I_fixed,P_fixed,V_rot,I_rot,P_rot
```
</details>

---

<details>
<summary>Python Dashboard</summary> 

**📂 Path:** `Python/dashboard.py`

**Install Dependencies:**
```bash
pip install -r requirements.txt
```

**Contents of `requirements.txt`:**

```
pyqt5
pyqtgraph
pyserial
pandas
```

**Functions:**

* Reads serial data in real-time.
* Plots **power vs. time** using `PyQtGraph`.
* Computes Power Gain:
      ![Gain Formula](https://latex.codecogs.com/svg.image?\text{Gain(%%)}%20=%20\frac{P_{rot}%20-%20P_{fixed}}{P_{fixed}}%20\times%20100)

* Supports simulation mode if Arduino not connected.

**Run Dashboard:**

```bash
cd Python
python dashboard.py
```

</details>

## System Architecture

###  Block Diagram

![Block Diagram](images/block_diagram.jpg)

### Circuit Diagram

![Circuit Diagram](images/circuit_diagram.png)

###  Hardware Prototype

![Hardware Setup](images/hardware_setup.png)

###  Python Dashboard
![ Python Dashboard](images/python.jpg)

---

## Contributors


| [![Kaarmukilan A G](https://img.shields.io/badge/KaarmukilanAG-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/Kaarmukilan17) | [![Dhruv Patel](https://img.shields.io/badge/DhruvPatel-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/dhruvpatel-vit) | [![Souvik Kundu Poddar](https://img.shields.io/badge/SouvikKunduPoddar-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/souvikkundu23) | [![Sankesh S U](https://img.shields.io/badge/SankeshSU-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/Sankesh005) |
|

---