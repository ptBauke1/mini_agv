import serial
import time

# Configuration
SERIAL_PORT = 'COM3'  # Change this to your Bluetooth serial port (COM3, COM4, etc.)
BAUD_RATE = 9600
TIMEOUT = 1

def parse_telemetry(line):
    """
    Parse telemetry message format: L:<left_pwm>,R:<right_pwm>,E:<error>,D:<distance>
    Returns dictionary with parsed values or None if parsing fails
    """
    try:
        # Remove whitespace and newline
        line = line.strip()
        
        # Split by comma
        parts = line.split(',')
        
        if len(parts) != 4:
            return None
        
        # Extract values
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
        print(f"Parse error: {e} - Line: {line}")
        return None

def main():
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        # Open serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print("Connected! Waiting for data...\n")
        
        while True:
            if ser.in_waiting > 0:
                # Read line from serial
                line = ser.readline().decode('utf-8', errors='ignore')
                
                # Parse the telemetry data
                data = parse_telemetry(line)
                
                if data:
                    # Print formatted output
                    print(f"[{time.strftime('%H:%M:%S')}] "
                          f"Left PWM: {data['left_pwm']:4d} | "
                          f"Right PWM: {data['right_pwm']:4d} | "
                          f"Error: {data['error']:6.2f} | "
                          f"Distance: {data['distance']:4d} mm")
                else:
                    # Print raw line if parsing fails
                    print(f"Raw: {line.strip()}")
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("\nTip: Make sure the COM port is correct and not in use by another program.")
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
