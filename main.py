import serial
import struct
import time
import pygame
import sys
import math

# --- Quick Configuration ---
SERIAL_PORT = 'COM11'  # Change this to your Arduino's serial port
BAUD_RATE = 115200

# --- Trim Configuration ---
ELEVATOR_TRIM = 0  # Elevator trim adjustment (-30 to +30)
RUDDER_TRIM = 0    # Rudder trim adjustment (-30 to +30)

# --- Flaps Configuration ---
FLAPS_TAKEOFF_OFFSET = 20  # Takeoff flaps: 20 degrees down
FLAPS_LANDING_OFFSET = 10  # Landing flaps: 10 degrees down

# --- Pygame Configuration ---
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 720
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GRAY = (128, 128, 128)

# --- PythonPacket (Data to Send to Arduino) ---
# Define the format string for the PythonPacket
# '<' for little-endian byte order (common for Arduino AVR)
# 'B' for uint8_t (autostabilize, throttle, checksum)
# 'b' for int8_t (aileronL, aileronR, elevator, rudder)
PYTHON_PACKET_FORMAT = '<BBbbbbB'
# Define the format string for the DATA part of the PythonPacket (without checksum)
PYTHON_PACKET_DATA_FORMAT = '<BBbbbb'

# --- PythonRecievePacket (Data to Receive from Arduino) ---
# Define the format string for the PythonRecievePacket
# 'f' for float (roll, pitch, yaw, voltage)
# 'B' for uint8_t (throttleActual, checksum)
# 'b' for int8_t (aileronLActual, aileronRActual, elevatorActual, rudderActual)
PYTHON_RECEIVE_PACKET_FORMAT = '<ffffBbbbbB'
# Define the format string for the DATA part of the PythonRecievePacket (without checksum)
PYTHON_RECEIVE_DATA_FORMAT = '<ffffBbbbb'

# Get the expected size of the receive packet for efficient reading
receive_packet_size = struct.calcsize(PYTHON_RECEIVE_PACKET_FORMAT)

# --- Checksum Calculation Function (Matches Arduino's XOR logic) ---
def calculate_checksum(data_bytes):
    """Calculates an XOR checksum for a given bytes object."""
    checksum_sum = 0
    for byte_val in data_bytes:
        checksum_sum ^= byte_val
    return checksum_sum % 256 # Ensure checksum fits in a uint8_t (0-255)

# --- Open Serial Port ---
def open_serial_port(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate) # Set a small timeout for non-blocking reads
        time.sleep(2)  # Give time for the connection to establish
        print(f"Connected to Arduino on {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}. {e}")
        print("Please ensure Arduino is connected and the port is correct.")
        exit()

def connect_controller():
    if pygame.joystick.get_count() == 0:
        print("No controller found")
        return
        
    xboxController = pygame.joystick.Joystick(0)
    xboxController.init()
    print(f"Connected to controller: {xboxController.get_name()}")
    print(f"Axes:` {xboxController.get_numaxes()}, Buttons: {xboxController.get_numbuttons()}")
    return xboxController

def send_data(ser, autostabilize, throttle, aileronL, aileronR, elevator, rudder):
    """Send data to Arduino via serial connection"""
    try:
        # 1. Pack the DATA part of the packet (excluding the checksum field)
        data_part_bytes_to_send = struct.pack(
            PYTHON_PACKET_DATA_FORMAT,
            autostabilize,
            throttle,
            aileronL,
            aileronR,
            elevator,
            rudder
        )
        
        # 2. Calculate the checksum on these data bytes
        calculated_checksum_send = calculate_checksum(data_part_bytes_to_send)
        
        # 3. Now, pack the ENTIRE packet, including the calculated checksum
        packed_full_packet_to_send = struct.pack(
            PYTHON_PACKET_FORMAT,
            autostabilize,
            throttle,
            aileronL,
            aileronR,
            elevator,
            rudder,
            calculated_checksum_send # Use the calculated checksum here
        )
        
        # 4. Send the packed bytes over serial
        ser.write(packed_full_packet_to_send)
        
        return True
    except Exception as e:
        print(f"Error sending data: {e}")
        return False

def receive_data(ser):
    """Receive data from Arduino via serial connection"""
    try:
        # Check if enough bytes are available for a full PythonRecievePacket
        if ser.in_waiting >= receive_packet_size:
            # Read the exact number of bytes
            received_bytes = ser.read(receive_packet_size)
            
            # Unpack the bytes into a tuple
            # Use the format for the full packet, including checksum
            unpacked_received_data = struct.unpack(PYTHON_RECEIVE_PACKET_FORMAT, received_bytes)
            
            # Separate the actual data bytes from the received checksum
            # Slicing received_bytes up to the last byte for checksum calculation
            data_part_bytes_received = received_bytes[:-1]
            received_checksum = unpacked_received_data[-1] # The last element is the checksum

            # Calculate checksum on the received data part
            calculated_checksum_receive = calculate_checksum(data_part_bytes_received)
            
            # Verify the checksum
            if calculated_checksum_receive == received_checksum:
                # Assign values to variables for clarity
                roll, pitch, yaw, voltage, throttleActual, aileronLActual, aileronRActual, elevatorActual, rudderActual, _ = unpacked_received_data
                
                return {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'voltage': voltage,
                    'throttleActual': throttleActual,
                    'aileronLActual': aileronLActual,
                    'aileronRActual': aileronRActual,
                    'elevatorActual': elevatorActual,
                    'rudderActual': rudderActual
                }
            else:
                print(f"RECVD <- Checksum MISMATCH! Expected {calculated_checksum_receive}, Got {received_checksum}. Packet discarded.")
                # You might want to clear the buffer if you suspect bad data
                ser.flushInput() # Clears input buffer to try and re-sync
                
    except Exception as e:
        print(f"Error receiving data: {e}")
    
    return None

def draw_3d_orientation(screen, center_x, center_y, roll, pitch, yaw, scale=100):
    """Draw 3D orientation using colored arrows representing aircraft axes"""
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    # Define initial vectors for aircraft axes (in aircraft body frame)
    # X-axis (forward) - Red arrow
    # Y-axis (right wing) - Green arrow  
    # Z-axis (down) - Blue arrow
    vectors = {
        'x': [1, 0, 0],  # Forward (Red)
        'y': [0, 1, 0],  # Right (Green)
        'z': [0, 0, 1]   # Down (Blue)
    }
    
    # Rotation matrices
    # Roll rotation (around X-axis)
    roll_matrix = [
        [1, 0, 0],
        [0, math.cos(roll_rad), -math.sin(roll_rad)],
        [0, math.sin(roll_rad), math.cos(roll_rad)]
    ]
    
    # Pitch rotation (around Y-axis)
    pitch_matrix = [
        [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
        [0, 1, 0],
        [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
    ]
    
    # Yaw rotation (around Z-axis)
    yaw_matrix = [
        [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
        [math.sin(yaw_rad), math.cos(yaw_rad), 0],
        [0, 0, 1]
    ]
    
    def matrix_multiply_vector(matrix, vector):
        """Multiply a 3x3 matrix with a 3x1 vector"""
        result = [0, 0, 0]
        for i in range(3):
            for j in range(3):
                result[i] += matrix[i][j] * vector[j]
        return result
    
    def matrix_multiply(m1, m2):
        """Multiply two 3x3 matrices"""
        result = [[0 for _ in range(3)] for _ in range(3)]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    result[i][j] += m1[i][k] * m2[k][j]
        return result
    
    # Combine rotations (yaw * pitch * roll)
    combined_matrix = matrix_multiply(yaw_matrix, matrix_multiply(pitch_matrix, roll_matrix))
    
    # Transform each vector and draw
    colors = {'x': RED, 'y': GREEN, 'z': BLUE}
    
    for axis, vector in vectors.items():
        # Apply rotation
        rotated_vector = matrix_multiply_vector(combined_matrix, vector)
        
        # Project to 2D (simple orthographic projection, ignoring Z for now)
        end_x = center_x + rotated_vector[0] * scale
        end_y = center_y - rotated_vector[1] * scale  # Negative because screen Y is inverted
        
        # Draw arrow
        pygame.draw.line(screen, colors[axis], (center_x, center_y), (end_x, end_y), 3)
        
        # Draw arrowhead
        arrow_length = 10
        arrow_angle = 0.5
        
        # Calculate arrowhead points
        dx = end_x - center_x
        dy = end_y - center_y
        length = math.sqrt(dx*dx + dy*dy)
        
        if length > 0:
            # Normalize direction
            dx /= length
            dy /= length
            
            # Calculate arrowhead points
            arrow_x1 = end_x - arrow_length * (dx * math.cos(arrow_angle) - dy * math.sin(arrow_angle))
            arrow_y1 = end_y - arrow_length * (dy * math.cos(arrow_angle) + dx * math.sin(arrow_angle))
            arrow_x2 = end_x - arrow_length * (dx * math.cos(-arrow_angle) - dy * math.sin(-arrow_angle))
            arrow_y2 = end_y - arrow_length * (dy * math.cos(-arrow_angle) + dx * math.sin(-arrow_angle))
            
            pygame.draw.line(screen, colors[axis], (end_x, end_y), (arrow_x1, arrow_y1), 2)
            pygame.draw.line(screen, colors[axis], (end_x, end_y), (arrow_x2, arrow_y2), 2)

def draw_telemetry(screen, font, received_data, left_stick_x, left_stick_y, right_stick_x, right_stick_y, throttle_value, aileronL, aileronR, elevator, rudder, throttle_sent, autostabilize, engine_armed, flaps_state):
    """Draw telemetry information on the screen"""
    y_offset = 20
    line_height = 30
    
    # Control inputs
    text = font.render("JOYSTICK INPUTS:", True, WHITE)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Roll Input: {left_stick_x:.3f}", True, GREEN)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Pitch Input: {left_stick_y:.3f}", True, GREEN)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Yaw Input: {right_stick_x:.3f}", True, GREEN)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Throttle Rate: {right_stick_y:.3f}", True, GREEN)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Current Throttle: {throttle_value:.1f}%", True, GREEN)
    screen.blit(text, (20, y_offset))
    y_offset += line_height * 2
    
    # Servo values being sent
    text = font.render("SERVO VALUES SENT:", True, WHITE)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Aileron L: {aileronL}", True, (255, 255, 0))  # Yellow
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Aileron R: {aileronR}", True, (255, 255, 0))  # Yellow
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Elevator: {elevator}", True, (255, 255, 0))  # Yellow
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Rudder: {rudder}", True, (255, 255, 0))  # Yellow
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    text = font.render(f"Throttle: {throttle_sent}", True, (255, 255, 0))  # Yellow
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    # Autostabilization status
    autostab_color = GREEN if autostabilize else RED
    autostab_text = "ON" if autostabilize else "OFF"
    text = font.render(f"Autostabilize: {autostab_text} (X to toggle)", True, autostab_color)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    # Engine armed status
    engine_color = GREEN if engine_armed else RED
    engine_text = "ARMED" if engine_armed else "DISARMED"
    text = font.render(f"Engine: {engine_text} (B to toggle)", True, engine_color)
    screen.blit(text, (20, y_offset))
    y_offset += line_height
    
    # Flaps status
    flaps_names = ["RETRACTED", "TAKEOFF", "LANDING"]
    flaps_colors = [WHITE, (255, 255, 0), (255, 165, 0)]  # White, Yellow, Orange
    text = font.render(f"Flaps: {flaps_names[flaps_state]} (A to cycle)", True, flaps_colors[flaps_state])
    screen.blit(text, (20, y_offset))
    y_offset += line_height * 2
    
    # Aircraft telemetry
    if received_data:
        text = font.render("AIRCRAFT TELEMETRY:", True, WHITE)
        screen.blit(text, (20, y_offset))
        y_offset += line_height
        
        text = font.render(f"Roll: {received_data['roll']:.1f}°", True, BLUE)
        screen.blit(text, (20, y_offset))
        y_offset += line_height
        
        text = font.render(f"Pitch: {received_data['pitch']:.1f}°", True, BLUE)
        screen.blit(text, (20, y_offset))
        y_offset += line_height
        
        text = font.render(f"Yaw: {received_data['yaw']:.1f}°", True, BLUE)
        screen.blit(text, (20, y_offset))
        y_offset += line_height
        
        text = font.render(f"Voltage: {received_data['voltage']:.2f}V", True, BLUE)
        screen.blit(text, (20, y_offset))
        y_offset += line_height
        
        text = font.render(f"Throttle Actual: {received_data['throttleActual']}", True, BLUE)
        screen.blit(text, (20, y_offset))
        y_offset += line_height
        
        # Control surface positions from aircraft
        text = font.render("ACTUAL CONTROL SURFACES:", True, WHITE)
        screen.blit(text, (400, 20))
        
        text = font.render(f"Aileron L Actual: {received_data['aileronLActual']}", True, BLUE)
        screen.blit(text, (400, 50))
        
        text = font.render(f"Aileron R Actual: {received_data['aileronRActual']}", True, BLUE)
        screen.blit(text, (400, 80))
        
        text = font.render(f"Elevator Actual: {received_data['elevatorActual']}", True, BLUE)
        screen.blit(text, (400, 110))
        
        text = font.render(f"Rudder Actual: {received_data['rudderActual']}", True, BLUE)
        screen.blit(text, (400, 140))
        
        # Draw 3D orientation
        text = font.render("AIRCRAFT ORIENTATION:", True, WHITE)
        screen.blit(text, (700, 20))
        
        text = font.render("Red = Forward", True, RED)
        screen.blit(text, (700, 50))
        
        text = font.render("Green = Right Wing", True, GREEN)
        screen.blit(text, (700, 80))
        
        text = font.render("Blue = Down", True, BLUE)
        screen.blit(text, (700, 110))
        
        # Draw the 3D orientation display
        draw_3d_orientation(screen, 800, 250, 
                          received_data['roll'], 
                          received_data['pitch'], 
                          received_data['yaw'])
        
        # Draw reference circle
        pygame.draw.circle(screen, GRAY, (800, 250), 110, 2)

def main():
    """Main application loop"""
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()
    
    # Set up display
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Plane Function Interface")
    font = pygame.font.Font(None, 24)
    clock = pygame.time.Clock()
    
    # Initialize serial and controller
    ser = open_serial_port(port=SERIAL_PORT, baud_rate=BAUD_RATE)
    xboxController = connect_controller()
    
    # Basic variables
    autostabilize = 0
    throttle = 0
    aileronL = 0
    aileronR = 0
    elevator = 0
    rudder = 0
    received_data = None
    
    # Throttle control variables
    throttle_rate = 0  # Current throttle percentage (0-100)
    max_throttle_change_rate = 2  # Max change per frame
    
    # Button state tracking
    x_button_pressed_last_frame = False
    b_button_pressed_last_frame = False
    a_button_pressed_last_frame = False
    l1_button_pressed_last_frame = False
    
    # System states
    engine_armed = False
    flaps_state = 0  # 0 = retracted, 1 = takeoff, 2 = landing
    auto_calibration_pending = False  # Flag for one-time autostabilize=2 packet
    
    running = True
    
    try:
        while running:
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # Update joystick events
            pygame.event.pump()
            
            # Read Xbox controller axes
            left_stick_x = xboxController.get_axis(0)    # Roll input
            left_stick_y = xboxController.get_axis(1)    # Pitch input
            right_stick_x = xboxController.get_axis(2)   # Yaw input
            right_stick_y = xboxController.get_axis(3)   # Throttle rate input

            # Handle X button for autostabilization toggle
            x_button_pressed = xboxController.get_button(2)  # X button is typically button 2
            if x_button_pressed and not x_button_pressed_last_frame:
                # Button was just pressed (rising edge)
                autostabilize = 1 if autostabilize == 0 else 0
                print(f"Autostabilization {'ON' if autostabilize else 'OFF'}")
            x_button_pressed_last_frame = x_button_pressed
            
            # Handle B button for engine arming toggle
            b_button_pressed = xboxController.get_button(1)  # B button is typically button 1
            if b_button_pressed and not b_button_pressed_last_frame:
                # Button was just pressed (rising edge)
                engine_armed = not engine_armed
                print(f"Engine {'ARMED' if engine_armed else 'DISARMED'}")
            b_button_pressed_last_frame = b_button_pressed
            
            # Handle A button for flaps control
            a_button_pressed = xboxController.get_button(0)  # A button is typically button 0
            if a_button_pressed and not a_button_pressed_last_frame:
                # Button was just pressed (rising edge)
                flaps_state = (flaps_state + 1) % 3  # Cycle through 0, 1, 2
                flaps_names = ["RETRACTED", "TAKEOFF", "LANDING"]
                print(f"Flaps: {flaps_names[flaps_state]}")
            a_button_pressed_last_frame = a_button_pressed

            # Handle L1 button for auto calibration (autostabilize = 2 for one packet)
            l1_button_pressed = xboxController.get_button(4)  # L1 button is typically button 4
            if l1_button_pressed and not l1_button_pressed_last_frame:
                # Button was just pressed (rising edge)
                auto_calibration_pending = True
                print("Auto calibration sent")
            l1_button_pressed_last_frame = l1_button_pressed

            # Map controller inputs to aircraft controls
            # Roll: Left stick X (-60 to +60 degrees)
            aileronL = int(left_stick_x * 60)   # Left aileron
            aileronR = int(left_stick_x * -60)  # Right aileron (opposite)
            
            # Pitch: Left stick Y (-30 to +30 degrees)
            elevator = int(-left_stick_y * 30)  # Negative because stick up = nose up = negative elevator
            
            # Yaw: Right stick X (-30 to +30 degrees)
            rudder = int(right_stick_x * 30)
            
            # Apply trim adjustments
            elevator += ELEVATOR_TRIM
            rudder += RUDDER_TRIM
            
            # Apply flaps offset to ailerons
            flaps_offset = 0
            if flaps_state == 1:  # Takeoff
                flaps_offset = FLAPS_TAKEOFF_OFFSET
            elif flaps_state == 2:  # Landing
                flaps_offset = FLAPS_LANDING_OFFSET
                
            aileronL += flaps_offset
            aileronR += flaps_offset
            
            # Clamp aileron values to safe ranges
            aileronL = max(-90, min(90, aileronL))
            aileronR = max(-90, min(90, aileronR))
            
            # Clamp other values to safe ranges
            elevator = max(-30, min(30, elevator))
            rudder = max(-30, min(30, rudder))
            
            # Throttle: Right stick Y controls throttle rate (not direct control)
            throttle_input = -right_stick_y  # Negative because stick up = increase throttle
            throttle_change = round(throttle_input, 1) * max_throttle_change_rate

            # Update throttle rate with limits
            throttle_rate += throttle_change
            throttle_rate = max(0, min(100, throttle_rate))  # Clamp between 0-100
            
            # Convert throttle percentage to the range your system expects
            # If engine is disarmed, force throttle to 0
            throttle = int(throttle_rate) if engine_armed else 0

            # Determine autostabilize value for this packet
            autostabilize_to_send = autostabilize
            if auto_calibration_pending:
                autostabilize_to_send = 2
                auto_calibration_pending = False  # Reset flag after one packet

            # Send data to Arduino
            send_data(ser, autostabilize_to_send, throttle, aileronL, aileronR, elevator, rudder)
            
            # Receive data from Arduino
            received_data = receive_data(ser)
            
            # Clear screen
            screen.fill(BLACK)
            
            # Draw telemetry
            draw_telemetry(screen, font, received_data, left_stick_x, left_stick_y, 
                          right_stick_x, right_stick_y, throttle_rate, 
                          aileronL, aileronR, elevator, rudder, throttle, autostabilize, engine_armed, flaps_state)
            
            # Update display
            pygame.display.flip()
            clock.tick(20)  # 20 FPS to match the original sleep timing
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    main()

