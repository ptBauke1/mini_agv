import serial
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

# Configuration
SERIAL_PORT = 'COM3'  # Change this to your Bluetooth serial port
BAUD_RATE = 9600
TIMEOUT = 1
MAX_POINTS = 100  # Number of data points to display in graphs

# Data storage
timestamps = deque(maxlen=MAX_POINTS)
left_pwm_data = deque(maxlen=MAX_POINTS)
right_pwm_data = deque(maxlen=MAX_POINTS)
error_data = deque(maxlen=MAX_POINTS)
distance_data = deque(maxlen=MAX_POINTS)

# Current values (for display)
current_values = {
    'left_pwm': 0,
    'right_pwm': 0,
    'error': 0.0,
    'distance': 0
}

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
            'distance': distance
        }
    except (ValueError, IndexError):
        return None

def read_serial_data():
    """Read and parse data from serial port"""
    global current_values
    
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore')
            data = parse_telemetry(line)
            
            if data:
                current_time = time.time()
                
                # Update current values
                current_values = data
                
                # Append to history
                timestamps.append(current_time)
                left_pwm_data.append(data['left_pwm'])
                right_pwm_data.append(data['right_pwm'])
                error_data.append(data['error'])
                distance_data.append(data['distance'])
        except Exception as e:
            print(f"Error reading serial: {e}")

def init_plot():
    """Initialize the plot"""
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle('AGV Telemetry Dashboard', fontsize=16, fontweight='bold')
    
    # Create grid layout
    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.3)
    
    # Motor PWM plot
    ax1 = fig.add_subplot(gs[0, :])
    ax1.set_title('Motor PWM Values')
    ax1.set_ylabel('PWM')
    ax1.set_xlabel('Time (s)')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-1000, 1000)
    
    # Line error plot
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.set_title('Line Following Error')
    ax2.set_ylabel('Error')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    
    # Distance plot
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.set_title('Ultrasonic Distance')
    ax3.set_ylabel('Distance (mm)')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True, alpha=0.3)
    
    # Current values display
    ax4 = fig.add_subplot(gs[2, :])
    ax4.axis('off')
    
    return fig, (ax1, ax2, ax3, ax4)

def update_plot(frame, axes):
    """Update plot with new data"""
    ax1, ax2, ax3, ax4 = axes
    
    # Read new data
    read_serial_data()
    
    if len(timestamps) == 0:
        return
    
    # Convert timestamps to relative seconds
    start_time = timestamps[0]
    time_axis = [t - start_time for t in timestamps]
    
    # Clear and update Motor PWM plot
    ax1.clear()
    ax1.set_title('Motor PWM Values')
    ax1.set_ylabel('PWM')
    ax1.set_xlabel('Time (s)')
    ax1.grid(True, alpha=0.3)
    ax1.plot(time_axis, left_pwm_data, label='Left Motor', color='blue', linewidth=2)
    ax1.plot(time_axis, right_pwm_data, label='Right Motor', color='green', linewidth=2)
    ax1.legend(loc='upper right')
    ax1.set_ylim(-1000, 1000)
    
    # Clear and update Error plot
    ax2.clear()
    ax2.set_title('Line Following Error')
    ax2.set_ylabel('Error')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    ax2.plot(time_axis, error_data, color='orange', linewidth=2)
    ax2.fill_between(time_axis, error_data, alpha=0.3, color='orange')
    
    # Clear and update Distance plot
    ax3.clear()
    ax3.set_title('Ultrasonic Distance')
    ax3.set_ylabel('Distance (mm)')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True, alpha=0.3)
    ax3.plot(time_axis, distance_data, color='red', linewidth=2)
    ax3.fill_between(time_axis, distance_data, alpha=0.3, color='red')
    
    # Update current values display
    ax4.clear()
    ax4.axis('off')
    
    # Create nice boxes for current values
    box_y = 0.5
    box_width = 0.22
    box_height = 0.8
    
    boxes = [
        {'x': 0.02, 'label': 'Left PWM', 'value': f"{current_values['left_pwm']}", 'color': '#3498db'},
        {'x': 0.27, 'label': 'Right PWM', 'value': f"{current_values['right_pwm']}", 'color': '#2ecc71'},
        {'x': 0.52, 'label': 'Error', 'value': f"{current_values['error']:.2f}", 'color': '#e74c3c'},
        {'x': 0.77, 'label': 'Distance', 'value': f"{current_values['distance']} mm", 'color': '#9b59b6'},
    ]
    
    for box in boxes:
        # Draw box
        rect = patches.FancyBboxPatch(
            (box['x'], box_y - box_height/2), box_width, box_height,
            boxstyle="round,pad=0.01", 
            linewidth=2, 
            edgecolor=box['color'], 
            facecolor=box['color'],
            alpha=0.2,
            transform=ax4.transAxes
        )
        ax4.add_patch(rect)
        
        # Add label
        ax4.text(box['x'] + box_width/2, box_y + 0.15, box['label'],
                ha='center', va='center', fontsize=10, fontweight='bold',
                transform=ax4.transAxes)
        
        # Add value
        ax4.text(box['x'] + box_width/2, box_y - 0.15, box['value'],
                ha='center', va='center', fontsize=14, fontweight='bold',
                color=box['color'], transform=ax4.transAxes)

def main():
    global ser
    
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        # Open serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print("Connected! Starting dashboard...\n")
        
        # Create plot
        fig, axes = init_plot()
        
        # Animate
        ani = FuncAnimation(fig, update_plot, fargs=(axes,), 
                          interval=100, cache_frame_data=False)
        
        plt.show()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("\nTip: Make sure the COM port is correct and not in use by another program.")
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
