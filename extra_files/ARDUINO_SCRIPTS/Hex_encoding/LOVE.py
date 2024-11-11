import serial
import time

# Replace 'COM13' with your actual port, e.g., '/dev/ttyUSB0' on Linux or 'COM3' on Windows
ser = serial.Serial('COM13', 115200, timeout=1)
time.sleep(2)  # Give time for the serial connection to initialize

# Flush any data left in the buffers
ser.reset_input_buffer()
ser.reset_output_buffer()

# Define the two bytes to send
start_byte = 0b00100010
upper_byte = 0x00  # Example value for the upper byte
lower_byte = 0x00  # Example value for the lower byte

# Set the frequency to 50 Hz (delay of 1/50 seconds per iteration)
delay = 1 / 800  # 0.02 seconds (50 Hz)

try:
    while True:
        # Read the comma returned by Arduino as a separator (optional, can be omitted if not used)
        ser.write(','.encode('UTF-8'))
        
        # Read the comma returned by Arduino as a separator (optional, can be omitted if not used)
        if ser.read() == b',':
            pass

        # Send the first byte
        ser.write(bytes([upper_byte]))
        
        # Read the comma returned by Arduino as a separator (optional, can be omitted if not used)
        if ser.read() == b',':
            pass
        
        # Send the second byte
        ser.write(bytes([lower_byte]))
        
        # Read the hexadecimal result returned by the Arduino
        result_hex = ser.readline().decode().strip()
        print(f"Received hex value from Arduino: {result_hex}")
        
        if lower_byte == 0xFF:
            lower_byte = 0
            if upper_byte == 0xFF:
                upper_byte = 0
            upper_byte += 1
            
        
        #Adding one to the result
        lower_byte = lower_byte + 1

        # Wait for the next iteration (50 Hz)
        time.sleep(delay)

finally:
    # Close the serial connection
    ser.close()
