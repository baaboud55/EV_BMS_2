# 🔋 EV Battery Management System (EV_BMS_2)

> **Senior Project** - College of Engineering and Islamic Architecture Studies (CEIES), King Abdulaziz University (KAU)

An advanced Battery Management System (BMS) for 8-cell Li-ion battery packs, built on the ESP32-S3 microcontroller with real-time monitoring, WiFi dashboard, and over-the-air (OTA) firmware updates.

---

## 📸 Dashboard Screenshots

<!-- Add your dashboard screenshots here -->
<!-- Example: ![Dashboard Overview](docs/images/dashboard.png) -->

*Screenshots coming soon - Place your GUI images in a `docs/images/` folder and update the links above*

---

## ✨ Features

### 🔌 Hardware Integration
- **ESP32-S3 DevKitC-1** microcontroller
- **8-cell Li-ion pack** monitoring (configurable: 8S1P or 2S4P)
- **ACS712 current sensor** with automatic calibration
- **Daisy-chained BMS slaves** for distributed voltage/temperature sensing
- **SD card** for data logging and persistence
- **Precharge circuit** for inrush current protection

### 📊 State Estimation
- **Kalman Filter SOC** (State of Charge) estimation
  - Coulomb counting with OCV correction
  - Temperature-compensated voltage-based calibration
- **Kalman Filter SOH** (State of Health) tracking
  - Capacity fade monitoring
  - Cycle counting

### 🛡️ Protection Features
- **Overvoltage Protection** (>4.2V per cell)
- **Undervoltage Protection** (<3.0V per cell)
- **Overcurrent Protection** (>50A)
- **Temperature Protection** (0°C - 45°C operating range)
- **Active Cell Balancing** support

### 🔄 State Machine
The BMS operates through a robust state machine:

```
┌─────────────┐
│   STARTUP   │
└──────┬──────┘
       │
       ▼
┌─────────────┐    Charger      ┌─────────────┐
│    IDLE     │───Connected────►│  CHARGING   │
└──────┬──────┘                 └─────────────┘
       │ Load Switch
       ▼
┌─────────────┐                 ┌─────────────┐
│ PRECHARGING │────3 sec───────►│ DISCHARGING │
└─────────────┘                 └─────────────┘
       │                               │
       └───────────┬───────────────────┘
                   │ Fault Condition
                   ▼
            ┌─────────────┐
            │    FAULT    │
            └─────────────┘
```

### 🌐 Web Dashboard
- **Real-time monitoring** via WebSocket streaming
- **Responsive design** - works on desktop and mobile
- **Interactive charts** using Chart.js
  - Individual cell voltages (8 cells)
  - Pack voltage over time
  - Current over time
  - SOC/SOH trending
- **Status indicators** for protection systems and faults

### 📡 Connectivity
- **WiFi Access Point/Station** mode
- **WebSocket** for real-time data streaming
- **REST API** (`/api/bms`) for data access
- **OTA Updates** - update firmware wirelessly

---

## 🔧 Hardware Setup

### Components Required
| Component | Specification | Quantity |
|-----------|---------------|----------|
| ESP32-S3 DevKitC-1 | Main controller | 1 |
| ACS712 Current Sensor | 20A or 30A variant | 1 |
| BMS Slave Boards | Custom daisy-chain | 2 |
| MicroSD Card Module | SPI interface | 1 |
| Precharge Relay | 12V coil | 1 |
| Load Contactor | High current | 1 |
| Charge MOSFET | P-channel | 1 |
| Li-ion Cells | 18650 or similar | 8 |

### Pin Connections

#### ESP32-S3 Pin Mapping

| Function | GPIO Pin | Description |
|----------|----------|-------------|
| **Current Sensor** | GPIO 1 | ACS712 analog output |
| **Voltage Feedback** | GPIO 19 | CV mode feedback |
| **Charger Sense** | GPIO 20 | Input voltage detection |
| **Precharge Relay** | GPIO 45 | Relay control |
| **Charge Enable** | GPIO 47 | PMOS gate control |
| **Load Enable** | GPIO 40 | Main contactor |
| **Load Switch** | GPIO 42 | User input switch |
| **PWM Output** | GPIO 48 | Charge current control |
| **BMS UART RX** | GPIO 17 | Slave communication |
| **BMS UART TX** | GPIO 16 | Slave communication |

#### SD Card SPI Connections

| Function | GPIO Pin |
|----------|----------|
| MISO | GPIO 13 |
| MOSI | GPIO 11 |
| SCK | GPIO 12 |
| CS | GPIO 10 |

### Wiring Diagram

```
                                    ┌──────────────────┐
                                    │   ESP32-S3       │
                                    │   DevKitC-1      │
                                    │                  │
    ┌───────────┐                   │  GPIO 1  ◄──────┼──── ACS712 OUT
    │  Li-ion   │                   │  GPIO 19 ◄──────┼──── Voltage Divider
    │  8S Pack  │                   │  GPIO 20 ◄──────┼──── Charger Detect
    │           │                   │                  │
    │  Cell 1-4 │◄──────────────────┼─── UART RX (17) │
    │  (Slave1) │                   │                  │
    │           │──────────────────►┼─── UART TX (16) │
    │  Cell 5-8 │                   │                  │
    │  (Slave2) │                   │  GPIO 45 ───────┼────► Precharge Relay
    └───────────┘                   │  GPIO 47 ───────┼────► Charge PMOS
          │                         │  GPIO 40 ───────┼────► Load Contactor
          │                         │  GPIO 42 ◄──────┼──── Load Switch
          ▼                         │                  │
    ┌───────────┐                   │  GPIO 10-13 ────┼────► SD Card
    │   Load    │                   │                  │
    │  (Motor/  │                   └──────────────────┘
    │  Inverter)│                          │
    └───────────┘                          │ WiFi
                                           ▼
                                    ┌──────────────────┐
                                    │  Web Dashboard   │
                                    │  (Browser)       │
                                    └──────────────────┘
```

---

## 🚀 Quick Start

### Prerequisites
- [PlatformIO](https://platformio.org/) (VS Code extension recommended)
- Python 3.8+ (for simulation)
- ESP32-S3 DevKitC-1 board

### 1. Clone the Repository
```bash
git clone https://github.com/baaboud55/EV_BMS_2.git
cd EV_BMS_2
```

### 2. Configure WiFi
Edit `src/main.cpp` and update your WiFi credentials:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### 3. Build and Upload

#### First Upload (USB Required)
```bash
# Build firmware
pio run

# Upload via USB
pio run --target upload

# Upload filesystem (web dashboard)
pio run --target uploadfs
```

#### Subsequent Updates (OTA)
See [OTA_USAGE.md](OTA_USAGE.md) for wireless update instructions.

### 4. Access the Dashboard
1. Open Serial Monitor to find the IP address:
   ```bash
   pio device monitor
   ```
2. Look for: `IP address: 192.168.x.x`
3. Open browser and navigate to `http://192.168.x.x`

---

## 🖥️ Development & Testing

### Python Simulation GUI
Test the dashboard without hardware using the included simulator:

```bash
# Install dependencies
pip install websockets pandas openpyxl

# Run simulation
python simulate_gui.py
```

This will:
1. Start a local HTTP server on port 8000
2. Start a WebSocket server on port 8765
3. Stream simulated BMS data from the validation Excel file
4. Auto-open browser to `http://localhost:8000`

### Native Testing (PC)
Run unit tests without hardware:
```bash
# Uncomment native environment in platformio.ini first
pio test -e native --verbose
```

---

## 📁 Project Structure

```
EV_BMS_2/
├── src/
│   ├── main.cpp              # Main application & state machine
│   ├── KalmanFilter.cpp      # SOC/SOH Kalman filter implementation
│   ├── OTAManager.cpp        # Over-the-air update handler
│   ├── SDCardManager.cpp     # SD card logging and persistence
│   ├── SOCEstimator.cpp      # State of Charge estimation
│   ├── SOHEstimator.cpp      # State of Health estimation
│   └── BMSStateManager.cpp   # State machine management
├── include/
│   ├── BMSConfig.h           # Hardware configuration & parameters
│   ├── KalmanFilter.h        # Kalman filter headers
│   ├── OTAManager.h          # OTA configuration
│   ├── SDCardManager.h       # SD card interface
│   ├── ChargeController.h    # Charge control & states
│   └── ...
├── data/
│   ├── index.html            # Web dashboard HTML
│   ├── style.css             # Dashboard styling
│   └── script.js             # Real-time charts & WebSocket
├── test/
│   ├── mocks/                # Hardware mocks for PC testing
│   └── test_simulation.cpp   # Unit tests
├── platformio.ini            # PlatformIO configuration
├── simulate_gui.py           # Python WebSocket simulator
├── OTA_USAGE.md              # OTA update guide
└── OTA_QUICK_REFERENCE.md    # OTA quick reference
```

---

## 📡 API Reference

### WebSocket Endpoint
- **URL**: `ws://<device-ip>/ws`
- **Data Format**: JSON

#### Sample Payload
```json
{
  "packVoltage": 28.5,
  "current": -2.5,
  "soc": 75.2,
  "soh": 98.5,
  "batteryState": "CHARGING",
  "cellVoltages": [3.56, 3.57, 3.55, 3.56, 3.57, 3.55, 3.56, 3.57],
  "slaveTemperatures": [25.3, 26.1],
  "prechargeActive": false,
  "protectionPMOSActive": true,
  "activeBalancingActive": false,
  "overvoltage": false,
  "undervoltage": false,
  "overcurrent": false,
  "remainingRuntime": 5.2,
  "cumulativeCapacity": 4.5,
  "timestamp": 1699123456
}
```

### REST API
- **GET** `/api/bms` - Returns current BMS data (same format as WebSocket)
- **GET** `/update` - OTA firmware update page

---

## ⚙️ Configuration

Key parameters in `include/BMSConfig.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUM_CELLS` | 8 | Number of battery cells |
| `BATTERY_CAPACITY` | 6.0 Ah | Nominal pack capacity |
| `MAX_CELL_VOLTAGE` | 4.2V | Overvoltage threshold |
| `MIN_CELL_VOLTAGE` | 3.0V | Undervoltage threshold |
| `MAX_CURRENT` | 50A | Overcurrent threshold |
| `MAX_TEMP` | 45°C | High temperature cutoff |
| `MIN_TEMP` | 0°C | Low temperature cutoff |
| `PRECHARGE_TIME_MS` | 3000ms | Precharge duration |

---

## 📚 Documentation

- [OTA Update Guide](OTA_USAGE.md) - Complete OTA firmware update instructions
- [OTA Quick Reference](OTA_QUICK_REFERENCE.md) - Quick reference card for OTA

---

## 👥 Team

This project was developed as a **Senior Project** at the College of Engineering and Islamic Architecture Studies (CEIES), King Abdulaziz University (KAU).

### Team Members
- **Mohammad Baaboud** - [@baaboud55](https://github.com/baaboud55)
- **Mohammad Samkari**
- **Mohammad Alsaiary**

---

## 🔮 Future Improvements

- [ ] Add Bluetooth Low Energy (BLE) support
- [ ] Implement predictive maintenance alerts
- [ ] Add multi-pack support
- [ ] Cloud data logging integration
- [ ] Mobile app development

---

## 📄 License

*License to be added*

---

## 🙏 Acknowledgments

- King Abdulaziz University (KAU)
- College of Engineering and Islamic Architecture Studies (CEIES)
- PlatformIO and ESP-IDF communities
- Chart.js for data visualization

---

<p align="center">
  Made with ❤️ at King Abdulaziz University
</p>
