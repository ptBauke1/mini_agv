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

# Custom HTTP handler to suppress favicon errors
class QuietHTTPHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Only log non-favicon requests
        if 'favicon.ico' not in args[0]:
            super().log_message(format, *args)
    
    def do_GET(self):
        # Return 204 No Content for favicon requests
        if self.path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return
        super().do_GET()

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

async def websocket_handler(websocket, clients):
    """Handle WebSocket connections"""
    clients.add(websocket)
    print(f"Client connected. Total clients: {len(clients)}")
    
    try:
        async for message in websocket:
            # Handle incoming messages from dashboard
            try:
                data = json.loads(message)
                
                # Handle COM port connection request
                if data.get('type') == 'connect_com':
                    global ser, SERIAL_PORT
                    new_port = data.get('com_port', SERIAL_PORT)
                    
                    # Close existing connection
                    if ser and ser.is_open:
                        ser.close()
                        print(f"Closed connection to {SERIAL_PORT}")
                    
                    # Try to connect to new port
                    try:
                        ser = serial.Serial(new_port, BAUD_RATE, timeout=1)
                        SERIAL_PORT = new_port
                        print(f"✓ Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': f'Connected to {SERIAL_PORT}'
                        }))
                    except serial.SerialException as e:
                        print(f"✗ Failed to connect to {new_port}: {e}")
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': f'Failed to connect to {new_port}: {str(e)}'
                        }))
                
                # Handle COM port disconnection request
                elif data.get('type') == 'disconnect_com':
                    if ser and ser.is_open:
                        ser.close()
                        print(f"Disconnected from {SERIAL_PORT}")
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': f'Disconnected from {SERIAL_PORT}'
                        }))
                    else:
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': 'No COM port was connected'
                        }))
                
                # Handle legacy config (for backward compatibility)
                elif data.get('type') == 'config':
                    new_port = data.get('com_port', SERIAL_PORT)
                    if ser and ser.is_open:
                        ser.close()
                    try:
                        ser = serial.Serial(new_port, BAUD_RATE, timeout=1)
                        SERIAL_PORT = new_port
                        print(f"✓ Connected to {SERIAL_PORT}")
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': f'Connected to {SERIAL_PORT}'
                        }))
                    except serial.SerialException as e:
                        print(f"✗ Failed to connect: {e}")
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': f'Failed to connect to {new_port}'
                        }))
                
                # Handle robot commands
                elif 'command' in data:
                    command = data['command']
                    print(f"Received command: {command}")
                    
                    # Send command to microcontroller via serial
                    if ser and ser.is_open:
                        command_str = f"{command}\n"
                        ser.write(command_str.encode('utf-8'))
                        print(f"Sent to MCU: {command_str.strip()}")
                    else:
                        print("Serial not connected, command not sent")
                        await websocket.send(json.dumps({
                            'type': 'status',
                            'message': 'Cannot send command: COM port not connected'
                        }))
                        
            except json.JSONDecodeError:
                print(f"Invalid JSON received: {message}")
    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        # Suppress common websocket handshake errors
        if not isinstance(e, (EOFError, websockets.exceptions.InvalidMessage)):
            print(f"WebSocket error: {e}")
    finally:
        clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(clients)}")

async def main():
    global ser
    
    # Set of connected WebSocket clients
    clients = set()
    
    # Start HTTP server in a separate thread
    os.chdir(os.path.dirname(os.path.abspath(__file__)) or '.')
    http_server = HTTPServer(('localhost', HTTP_PORT), QuietHTTPHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    print(f"HTTP server started on http://localhost:{HTTP_PORT}")
    
    # Open serial port with retry
    max_retries = 3
    for attempt in range(max_retries):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"✓ Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
            break
        except serial.SerialException as e:
            if attempt < max_retries - 1:
                print(f"Attempt {attempt + 1}/{max_retries} failed: {e}")
                print(f"Retrying in 2 seconds...")
                await asyncio.sleep(2)
            else:
                print(f"\n✗ Serial error after {max_retries} attempts: {e}")
                print("\nTroubleshooting:")
                print("  1. Check if another program is using COM3 (Arduino IDE, PuTTY, etc.)")
                print("  2. Try unplugging and replugging the Bluetooth module")
                print("  3. Check Bluetooth pairing in Windows settings")
                print("  4. Try a different COM port\n")
                print("Continuing without serial connection...")
    
    # Start WebSocket server
    async with websockets.serve(
        lambda ws: websocket_handler(ws, clients),
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
