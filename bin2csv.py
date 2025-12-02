import struct
import csv

# --- CONFIGURATION ---
INPUT_FILENAME = 'response.bin'  # The file you downloaded from ESP32
OUTPUT_FILENAME = 'sensor_data.csv' # The file you want to create
# '7f' matches the ESP32 code: 7 floats (ax, ay, az, gx, gy, gz, temp)
DATA_FORMAT = '7f' 
# ---------------------

def convert_binary_to_csv():
    record_size = struct.calcsize(DATA_FORMAT)
    print(f"Reading {INPUT_FILENAME}...")
    
    try:
        with open(INPUT_FILENAME, 'rb') as bin_file, open(OUTPUT_FILENAME, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            
            # 1. Write the Header (Labels for your columns)
            writer.writerow(['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp'])
            
            count = 0
            while True:
                # 2. Read exactly one chunk of data (28 bytes)
                chunk = bin_file.read(record_size)
                
                # Stop if we reach the end of the file
                if not chunk:
                    break
                    
                # Stop if we get a corrupted/partial chunk
                if len(chunk) != record_size:
                    print("Warning: Incomplete chunk found at end of file.")
                    break
                
                # 3. Unpack the binary back into numbers
                # The result is a tuple: (ax, ay, az, gx, gy, gz, temp)
                data = struct.unpack(DATA_FORMAT, chunk)
                
                # 4. Write to CSV
                writer.writerow(data)
                count += 1
                
        print(f"Success! Converted {count} samples to {OUTPUT_FILENAME}")
        
    except FileNotFoundError:
        print(f"Error: Could not find '{INPUT_FILENAME}'. Make sure it is in the same folder.")

if __name__ == "__main__":
    convert_binary_to_csv()