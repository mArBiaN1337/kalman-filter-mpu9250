import network
import socket
import os
import machine, sys
from mpu9250 import MPU9250
from boot import Boot
from time import sleep
# pyright: reportMissingImports=false

FILE_NAME = "imu_data.txt"
MAX_FILE_SIZE = 1 * 1024 * 1024  # 1 MB
SAMPLES_PER_DATASET = 250
SAMPLING_PERIOD = 0.01  # seconds

class IMUServer:
    def __init__(self):

        boot = Boot()
        
        self.btn_new_dataset = boot.btn1
        self.led_data_being_collected = boot.led1 
        self.led_data_available = boot.led2
        self.onboard_led = boot.onboard_led

        self.imu_data = ""
        self.file_size = 0

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

        self.i2c = machine.I2C(0, scl=machine.Pin(22), sda=machine.Pin(21), freq=400000)
        self.mpu = MPU9250(self.i2c)
        
        self.connect_wifi()

        self.collect_sensor_data()

        #self.btn_new_dataset.irq(trigger=machine.Pin.IRQ_FALLING, handler=self.collect_sensor_data)
        
        self.create_socket()

    def check_file_size_limit(self, file_name=FILE_NAME, max_size=MAX_FILE_SIZE):
        try:
            if file_name in os.listdir():

                file_size = os.stat(file_name)[6]
                if file_size >= max_size:

                    print(f"File '{file_name}' exceeds {max_size} bytes. Deleting it.")
                    os.remove(file_name)
                    self.imu_data = ""
                    self.file_size = 0
                    
                    with open(file_name, "w") as f:
                        pass
                else:
                    print("File size is OK.")

        except Exception as e:
            print(f"Error checking file size: {e}")

    def connect_wifi(self):
        self.port = 80
        
        with open('secret.txt', 'r') as f:
            self._net_ssid = f.readline().strip()
            self._net_pass = f.readline().strip()

        try:
            station = network.WLAN(network.STA_IF)
            if station.isconnected():
                print('Already connected to WiFi. IP:', station.ifconfig()[0])
                self.ip = station.ifconfig()[0]
                return
            else:
                print('Connecting to WiFi...')
                station.active(True)
                station.connect(self._net_ssid, self._net_pass)
                while not station.isconnected():
                    pass

                self.ip = station.ifconfig()[0]
                print('Connected to WiFi. IP:', self.ip)

        except Exception as e:
            print('Failed to connect to WiFi:', e)
            sys.exit()

    def collect_sensor_data(self, samples=SAMPLES_PER_DATASET, Ts=SAMPLING_PERIOD):

        self.check_file_size_limit()

        print("Collecting new dataset.")
        self.led_data_being_collected.value(1)
        self.led_data_available.value(0)

        with open(FILE_NAME, "w") as f:
            f.write("ax, ay, az, gx, gy, gz, temp\n")

        with open(FILE_NAME, "a") as f:
            for _ in range(samples):
                ax, ay, az = self.mpu.acceleration  # in m/s^2
                gx, gy, gz = self.mpu.gyro          # in rad/s
                temp = self.mpu.temperature         # in C

                # write data to file with commas and newline
                f.write("{:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}\n".format(ax, ay, az, gx, gy, gz, temp))
                f.flush()  # ensure data is written to file
                self.led_data_being_collected.toggle()
                sleep(Ts)

        self.update_file_content()
        self.onboard_led.value(1)
        self.led_data_being_collected.value(0)
        self.led_data_available.value(1)

    def update_file_content(self, file_name=FILE_NAME):
        try:
            self.file_size = 0
            with open(file_name, 'r') as f:
                self.imu_data = []
                while True:
                    line = f.readline()
                    if not line:
                        break
                    self.imu_data.append(line)
                    self.file_size += len(line)

                self.imu_data = ''.join(self.imu_data)
                print(f"File '{file_name}' loaded with size {self.file_size} bytes.")

        except Exception as e:
            print(f"Failed to open file '{file_name}': {e}")
            self.imu_data = "No data available"
            self.file_size = 0

    def create_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)

        self.sock.bind((self.ip, self.port))
        self.sock.listen(1)
        print(f"Listening on {self.ip}:{self.port}")

        self.onboard_led.value(0)
        
        while True:
            conn = None
            try:
                self.onboard_led.value(1)
                conn, addr = self.sock.accept()
                print(f"Connected by {addr}")                
                http_header = (
                        "HTTP/1.1 200 OK\r\n"
                        "Content-Type: text/csv\r\n"
                        f"Content-Length: {self.file_size}\r\n"
                        "Content-Disposition: attachment; filename=imu-data.csv\r\n"
                        "Connection: close\r\n"
                        "\r\n"
                    ).encode('utf-8')

                conn.sendall(http_header + self.imu_data.encode('utf-8'))
                conn.close()
                self.onboard_led.value(0)

            except OSError as e:
                pass 
                
            except KeyboardInterrupt:
                print("Shutting down connection.")
                self.onboard_led.value(0)
                if conn:
                    conn.close()
                
                raise 
        

if __name__ == "__main__":
    try:
        imu_server = IMUServer()

    except KeyboardInterrupt:
        print("IMU Server stopped.")

    finally:
        imu_server.onboard_led.value(0)
        if imu_server.sock:
            imu_server.sock.close()
        
