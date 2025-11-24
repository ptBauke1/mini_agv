# Bluetooth Telemetry Receiver

## Installation

1. Install Python packages:
```bash
pip install pyserial matplotlib websockets
```

## Usage Options

### 1. Simple Console Output (`bt_receiver.py`)

Quick view of telemetry in the terminal.

```bash
python bt_receiver.py
```

### 2. Interactive Matplotlib Dashboard (`bt_dashboard.py`)

Real-time graphs with matplotlib (desktop app).

```bash
python bt_dashboard.py
```

### 3. HTML Web Dashboard (`dashboard.html` + `bt_websocket_server.py`)

Modern web-based dashboard accessible from any browser!

**Steps:**
1. Edit `bt_websocket_server.py`:
   - Change `SERIAL_PORT = 'COM3'` to your actual port

2. Start the WebSocket server:
```bash
python bt_websocket_server.py
```

3. Open `dashboard.html` in your web browser

4. Click "Connect" button in the dashboard

**Features:**
- 📊 Real-time charts with Chart.js
- 💎 Modern, beautiful UI
- 📱 Responsive design
- 📝 Live data log
- 🎨 Color-coded metrics

## Configuration

For all scripts, find your Bluetooth COM port:
- Open Device Manager → Ports (COM & LPT)
- Look for your Bluetooth device (e.g., COM3, COM4)

## Output Example (Console)
```
[14:23:45] Left PWM:  650 | Right PWM:  750 | Error:  -1.23 | Distance:  245 mm
[14:23:46] Left PWM:  680 | Right PWM:  720 | Error:   0.45 | Distance:  243 mm
```

## Troubleshooting

- **"Access denied"**: Close any other program using the COM port
- **"Port not found"**: Check the COM port number in Device Manager
- **No data**: Make sure the Bluetooth connection is established
- **WebSocket error**: Ensure the server is running before opening the HTML dashboard
