# 📘 Formulae and Their Significance

This document explains all the key equations used in the **Fixed vs Rotating Solar Panel Power Comparison System**, along with their purpose and reasoning.

---

## ⚙️ 1. LDR Offset Calibration

### **Formula:**
![Offset Formula](https://latex.codecogs.com/svg.image?\text{offset}%20=%20\text{avg}(LDR1)%20-%20\text{avg}(LDR2))

### **Purpose:**
To compensate for **manufacturing differences** or **placement inaccuracies** in the two LDR sensors.

### **Explanation:**
Even under the same lighting, two LDRs may produce slightly different analog values due to:
- Variation in sensor resistance tolerance.
- Slight difference in placement or shielding.
- Non-linear behavior at extreme light intensities.

Hence, we calculate an average offset during calibration and subtract it during operation to ensure **balanced and accurate tracking**.

---

## 📈 2. Exponential Moving Average (EMA)

### **Formula:**
![EMA Formula](https://latex.codecogs.com/svg.image?S_t%20=%20\alpha%20D_t%20+%20(1%20-%20\alpha)S_{t-1})

### **Purpose:**
To **smooth fluctuations** in sensor readings caused by noise, sudden light changes, or ADC jitter.

### **Explanation:**
LDR readings tend to fluctuate rapidly when light intensity changes due to:
- Ambient reflections.
- Electrical noise in analog signals.
- Fast sunlight variation (clouds, shadows).

EMA acts as a low-pass filter that gives more weight to recent values (`α ≈ 0.15`) and less to older ones.  
This stabilizes the control loop, preventing the servo from oscillating.

---

## ⚡ 3. Power Calculation

### **Formula:**
![Power Formula](https://latex.codecogs.com/svg.image?P%20=%20V%20\times%20I)

### **Purpose:**
To compute **instantaneous electrical power output** of both panels.

### **Explanation:**
The INA219 sensors measure:
- Bus voltage (V)
- Current (I)

The product gives the **power in milliwatts (mW)** for both panels:
- `P_fixed` for the stationary panel.
- `P_rot` for the tracking panel.

This data is used for live comparison in the dashboard.

---

## 🌞 4. Power Gain Percentage

### **Formula:**
![Gain Formula](https://latex.codecogs.com/svg.image?\text{Gain(%%)}%20=%20\frac{P_{rot}%20-%20P_{fixed}}{P_{fixed}}%20\times%20100)

### **Purpose:**
To **quantify performance improvement** achieved by the solar tracker.

### **Explanation:**
It compares the power output of the rotating panel against the fixed panel.
A positive gain indicates the rotating panel is harvesting **more energy** by staying aligned to sunlight.

The gain is displayed in:
- The real-time PyQt5 dashboard gauge.
- The rolling average display for smoother trend tracking.

---

## 🔁 5. Servo Control Step (Simplified)

### **Formula:**
![Servo Step Formula](https://latex.codecogs.com/svg.image?\text{step}%20=%20\text{constrain}\left(\frac{\text{smoothDiff}}{\text{scale}},%20-%20\text{maxStep},%20\text{maxStep}\right))

### **Purpose:**
To determine **how much the servo motor should rotate** to realign the solar panel.

### **Explanation:**
- The `smoothDiff` (difference between LDR1 and LDR2) defines the tracking error.
- The servo adjusts in small, controlled steps (`maxStep = 5°`) to prevent overshoot.
- The `scale` factor (≈ 60) ensures smoother movement.

This algorithm maintains stable and continuous tracking.

---

## 🧠 Summary

| Formula | Variable | Purpose |
|----------|-----------|----------|
| Offset | `avg(LDR1) - avg(LDR2)` | Corrects sensor imbalance |
| EMA | `S_t = αD_t + (1-α)S_{t-1}` | Smooths noisy light readings |
| Power | `P = V × I` | Calculates instantaneous panel power |
| Power Gain | `(P_rot - P_fixed) / P_fixed × 100` | Evaluates tracker efficiency |
| Step Control | `step = smoothDiff / scale` | Controls servo rotation |

---

📚 *These equations together make the system accurate, responsive, and stable under changing sunlight conditions.*
