import serial
import csv
import time

# ============================================================
# CHANGE THIS PORT
# ============================================================

PORT = '/dev/cu.usbserial-1120'

# Examples:
# Windows  -> COM3
# Linux    -> /dev/ttyUSB0
# macOS    -> /dev/cu.usbmodemXXXX

BAUDRATE = 9600

OUTPUT_FILE = 'mcu_output.csv'

# ============================================================
# OPEN SERIAL PORT
# ============================================================

ser = serial.Serial(PORT, BAUDRATE)

time.sleep(2)

print("Connected to Arduino")
print("Saving CSV...")

# ============================================================
# OPEN CSV FILE
# ============================================================

with open(OUTPUT_FILE, 'w', newline='') as file:

    writer = csv.writer(file)

    while True:

        try:

            line = ser.readline().decode('utf-8').strip()

            if line:

                print(line)

                values = line.split(',')

                writer.writerow(values)

        except KeyboardInterrupt:

            print("Logging stopped")
            break

# ============================================================
# CLOSE SERIAL
# ============================================================

ser.close()