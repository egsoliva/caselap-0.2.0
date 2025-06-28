import serial
import openpyxl
import time

# Define Serial Port (Change this based on your setup)
SERIAL_PORT = "COM11"  # Windows Example: "COM11", Linux/Mac Example: "/dev/ttyUSB0"
BAUD_RATE = 9600

# Initialize Serial Communication
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Define Excel File Name
EXCEL_FILE = "gps_log.xlsx"

# Try to Load Existing Excel File, Otherwise Create a New One
try:
    workbook = openpyxl.load_workbook(EXCEL_FILE)
    sheet = workbook.active
except FileNotFoundError:
    workbook = openpyxl.Workbook()
    sheet = workbook.active
    sheet.append(["Timestamp", "Latitude", "Longitude", "Altitude", "Status"])

print("Logging GPS data from ESP8266 to Excel...")

# Initialize Variables
last_log_time = time.time()
lat, lon, alt, status = None, None, None, None

while True:
    try:
        line = ser.readline().decode("utf-8").strip()
        
        if line:
            print("Received:", line)

            # Extract GPS Data
            if "Latitude:" in line:
                lat = float(line.split(":")[1].strip())
            elif "Longitude:" in line:
                lon = float(line.split(":")[1].strip())
            elif "Altitude:" in line:
                alt = float(line.split(":")[1].strip())
            elif "Status:" in line:
                status = float(line.split(":")[1].strip())

            # Log Data Every 5 Seconds When All Values Are Available
            if lat is not None and lon is not None and alt is not None:
                current_time = time.time()
                if current_time - last_log_time >= 5:
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

                    sheet.append([timestamp, lat, lon, alt, status])
                    workbook.save(EXCEL_FILE)
                    print(f"Logged: {timestamp}, {lat}, {lon}, {alt}")
                    
                    # Update the last log time
                    last_log_time = current_time

    except KeyboardInterrupt:
        print("\nStopping logging...")
        ser.close()
        break
    except Exception as e:
        print("Error:", e)