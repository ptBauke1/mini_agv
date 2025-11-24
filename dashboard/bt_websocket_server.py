import asyncio
import websockets
import serial
import json
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os

# Configuration
SERIAL_PORT = 'COM3'  # Change this to your Bluetooth serial port
BAUD_RATE = 9600
WEBSOCKET_PORT = 8765
HTTP_PORT = 8080

# Serial connection
ser = None

def parse_telemetry(line):
    """Parse telemetry message format: L:<left_pwm>,R:<right_pwm>,E:<error>,D:<distance>"""
    try:
        line = line.strip()
        parts = line.split(',')
        
        if len(parts) != 4:
            return None
        
        left_pwm = int(parts[0].split(':')[1])
        right_pwm = int(parts[1].split(':')[1])
        error = float(parts[2].split(':')[1])
        distance = int(parts[3].split(':')[1])
        
        return {
            'left_pwm': left_pwm,
            'right_pwm': right_pwm,
            'error': error,
            'distance': distance,
            'timestamp': time.time()
        }
    except (ValueError, IndexError) as e:
        print(f"Parse error: {e}")
        return None

async def serial_reader(websocket_clients):
    """Read data from serial port and broadcast to all WebSocket clients"""
    global ser
    
    while True:
        try:
            if ser and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore')
                data = parse_telemetry(line)
                
                if data:
                    # Broadcast to all connected WebSocket clients
                    if websocket_clients:
                        message = json.dumps(data)
                        await asyncio.gather(
                            *[client.send(message) for client in websocket_clients],
                            return_exceptions=True
                        )
                        print(f"Sent: {message}")
        except Exception as e:
            print(f"Serial read error: {e}")
        
        await asyncio.sleep(0.01)

async def websocket_handler(websocket, path, clients):
    """Handle WebSocket connections"""
    clients.add(websocket)
    print(f"Client connected. Total clients: {len(clients)}")
    
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(clients)}")

async def main():
    global ser
    
    # Set of connected WebSocket clients
    clients = set()
    
    # Start HTTP server in a separate thread
    os.chdir(os.path.dirname(os.path.abspath(__file__)) or '.')
    http_server = HTTPServer(('localhost', HTTP_PORT), SimpleHTTPRequestHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    print(f"HTTP server started on http://localhost:{HTTP_PORT}")
    
    # Open serial port
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("Continuing without serial connection...")
    
    # Start WebSocket server
    async with websockets.serve(
        lambda ws, path: websocket_handler(ws, path, clients),
        "localhost",
        WEBSOCKET_PORT
    ):
        print(f"WebSocket server started on ws://localhost:{WEBSOCKET_PORT}")
        print(f"\n🚀 Open your browser and go to: http://localhost:{HTTP_PORT}/dashboard.html\n")
        
        # Start serial reader task
        await serial_reader(clients)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed")
