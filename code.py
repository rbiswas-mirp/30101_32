import serial
import time

# Replace with your ESP32 serial port
port = "/dev/ttyUSB0"  # or /dev/ttyACM0
baudrate = 115200

try:
    # Establish serial connection
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Allow time for the connection to be established
    
    print("ESP32 connection established")
    
    # Send restart command to turn off ESP32 initially
    restart_command = "restart\n"
    ser.write(restart_command.encode('utf-8'))
    print("ESP32 restart command sent - ESP32 is restarting")
    
    # Wait for 300 seconds while ESP32 is in restart/boot cycle
    print("Waiting 300 seconds...")
    time.sleep(300)
    
    # Send another restart command to ensure ESP32 is fully operational
    ser.write(restart_command.encode('utf-8'))
    print("Final restart command sent - ESP32 should be fully operational now")
    
    # Give ESP32 time to complete restart and begin normal operation
    time.sleep(5)
    
    # Now start reading data from ESP32
    print("Starting to read data from ESP32...")
    
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(f"Received value: {line}")
        time.sleep(0.1)

except serial.SerialException as e:
    print(f"Serial Error: {e}")
except KeyboardInterrupt:
    print("Script interrupted by user")
finally:
    # Close the connection when done
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed")
