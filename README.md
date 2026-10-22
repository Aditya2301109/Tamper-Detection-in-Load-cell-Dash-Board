# Tamper Detection in Load Cell Dashboard

An intelligent real-time monitoring and anti-tamper detection dashboard for electronic weighing scales and load cell systems. The application interfaces with microcontrollers (such as ESP32 or Arduino) via serial communication to stream raw ADC readings from an HX711 amplifier, applies multi-stage digital signal processing filters, and uses statistical deflection algorithms (CUSUM, slope monitoring, spike/step detection) to identify mechanical or electrical tampering while suppressing false alarms during legitimate weight placement.

---

## Key Features

- **Real-Time Data Streaming & GUI**: Built with Tkinter and Matplotlib (`TkAgg`), plotting live raw ADC values against filtered signals.
- **Advanced Digital Signal Processing (DSP)**:
  - **Savitzky-Golay Filter** for preserving peak shapes while smoothing noise.
  - **Butterworth Low-Pass Filter** (3rd order, configurable cutoff frequency).
  - **Simple Moving Average (SMA)**, **Exponential Smoothing (EMA)**, and **Median Filter**.
- **Multi-Algorithm Tamper Detection**:
  - **Spike & Step Detectors**: Identifies sudden transient shocks and unauthorized step offsets.
  - **Slope & CUSUM (Cumulative Sum)**: Detects subtle, progressive drift or slow manual tampering.
  - **Weight-Placement Settle Window**: Automatically detects intentional weight placements and temporarily suppresses alarms during mechanical settling.
  - **Startup Suppression & Alarm Cooldown**: Prevents false triggers on connection initialization and debounce periods.
  - **Baseline Calibration ("Learn Baseline")**: Dynamically computes threshold values tailored to the environment and hardware characteristics.
- **Automated Logging & Archiving**:
  - Automatically saves continuous time-series logs and tamper snapshots in both **CSV** and **Excel (.xlsx)** formats under `scale_logs/`.
  - Built-in automatic ZIP archiving for historical logs older than a configurable retention threshold.

---

## Hardware Architecture

```
[ Load Cell ] ---> [ HX711 24-Bit ADC ] ---> [ ESP32 / Arduino ] --- (USB Serial) ---> [ Python Dashboard ]
```

- **Load Cell**: Strain gauge weighing sensor (e.g., 5kg, 10kg, 50kg, or industrial scale load cells).
- **ADC Amplifier**: HX711 24-bit analog-to-digital converter.
- **Microcontroller**: ESP32, Arduino, STM32, or Raspberry Pi transmitting formatted serial readings.
- **Baud Rate**: Default `115200` baud.

---

## Repository Structure

```
Tamper-Detection-in-Load-cell-Dash-Board/
│
├── main.py               # Main application entry point & Tkinter GUI dashboard
├── requirements.txt      # Python dependencies
├── .gitignore            # Git exclusion rules
├── .gitattributes        # Repository path & line ending configuration
├── README.md             # Project documentation
└── scale_logs/           # Output directory for logs, snapshots, and zip archives
    ├── scale_log_20251211_235650.csv
    ├── scale_log_20251211_235650.xlsx
    ├── tamper_snapshot_20251212_000148.csv
    └── tamper_snapshot_20251212_000148.xlsx
```

---

## Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/Aditya2301109/Tamper-Detection-in-Load-cell-Dash-Board.git
cd Tamper-Detection-in-Load-cell-Dash-Board
```

### 2. Set Up a Virtual Environment (Recommended)
```bash
python -m venv env
# On Windows (PowerShell):
.\env\Scripts\Activate.ps1
# On Linux/macOS:
source env/bin/activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

---

## Configuration

Open `main.py` and adjust the user configuration section at the top of the file to match your hardware setup:

```python
# ---------------- USER SETTINGS ----------------
SERIAL_PORT = "COM13"       # Set to your microcontroller's serial port (e.g., COM3, /dev/ttyUSB0)
BAUD = 115200               # Serial baud rate
TIMEOUT = 1.0

# Sampling & Filter Parameters
SAMPLE_RATE_HZ = 50
SMA_WINDOW = 10
EXP_ALPHA = 0.08
MEDIAN_WINDOW = 5
SG_WINDOW = 11
SG_POLY = 2
BUTTERORDER = 3
BUTTER_CUTOFF_HZ = 2.0

# Settle Logic
SETTLE_AFTER_WEIGHT_SEC = 5.0    # Duration (seconds) to suppress alarms after placing a weight
WEIGHT_PLACEMENT_DELTA = 2000.0  # Count threshold to detect intentional placement
```

---

## Running the Application

Launch the dashboard:
```bash
python main.py
```

### Dashboard Controls:
- **Connect / Disconnect**: Starts or stops serial acquisition from the microcontroller.
- **Zero / Tare**: Zeroes the active scale reading.
- **Learn Baseline**: Samples steady-state noise to calibrate tamper thresholds automatically.
- **Acknowledge Alert**: Silences and resets active tamper warnings.
- **Export / Snapshot**: Exports instant snapshots to `scale_logs/` in CSV and XLSX format.

---

## License

This project is licensed under the MIT License.
