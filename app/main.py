import gc
import network
import socket
import os
import machine
import struct
from mpu9250 import MPU9250
from boot import Boot
from time import sleep
# pyright: reportMissingImports=false

FILE_NAME = "imu_data.bin"
SAMPLING_FREQUENCY = 100  # Hz
SAMPLING_PERIOD = 1.0 / SAMPLING_FREQUENCY  # seconds

class IMUServer:
    def __init__(self):

        boot = Boot()
        gc.collect()

        self.record_size = struct.calcsize('7f')  
        self.samples = 500
        self.total_bytes = self.record_size * self.samples
        self.buffer = bytearray(self.total_bytes)
        
        self.btn_new_dataset = boot.btn1
        self.led_data_being_collected = boot.led1 
        self.led_data_available = boot.led2
        self.onboard_led = boot.onboard_led
        
        self.imu_data = ""
        self.file_size = 0

        self.wtd = machine.WDT(timeout=20000)

        self.collect_flag = False
        self.sock = None
        self.state = 'IDLE'

        self.i2c = machine.I2C(0, scl=machine.Pin(22), sda=machine.Pin(21), freq=400000)
        self.mpu = MPU9250(self.i2c)

        self.turn_off_leds()
        
        self.connect_wifi()

        self.btn_new_dataset.irq(trigger=machine.Pin.IRQ_FALLING, handler=self.button_handler)


    def button_handler(self, pin):
        self.collect_flag = True

    def turn_off_leds(self):
        self.led_data_available.value(0)
        self.led_data_being_collected.value(0)

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

            self.blink_onboard_led()

        except Exception as e:
            print('Failed to connect to WiFi:', e)
            raise

    def blink_onboard_led(self, times=3, interval=0.2):
        for _ in range(times):
            self.onboard_led.value(1)
            sleep(interval)
            self.onboard_led.value(0)
            sleep(interval)

    def collect_sensor_data(self, Ts=SAMPLING_PERIOD):

        gc.collect()
        print("Collecting new dataset.")
        self.led_data_being_collected.value(1)
        self.led_data_available.value(0)

        offset = 0
        for _ in range(self.samples):
            ax, ay, az = self.mpu.acceleration  # in m/s^2
            gx, gy, gz = self.mpu.gyro          # in rad/s
            temp = self.mpu.temperature         # in C
            struct.pack_into('7f', self.buffer, offset, ax, ay, az, gx, gy, gz, temp)
            offset += self.record_size
            sleep(Ts)

        with open(FILE_NAME, "wb") as f:
            f.write(self.buffer)

        self.buffer = bytearray(self.total_bytes)
        self.led_data_being_collected.value(0)
        self.led_data_available.value(1)

        self.state = 'IDLE'
        print("Data collection complete.")
        # ip and port
        print(f"Server listening on {self.ip}:{self.port}")


    def poll_client(self):
        if not self.sock:
            return
        
        conn = None
        try:
            conn, addr = self.sock.accept()
            print(f"Connected by {addr}")
            try:  
                self.send_chunks(conn)
                sleep(0.1)

            except Exception as e:
                print(f"Error sending data: {e}")
            finally:
                conn.close()

        except OSError as e:
            if e.args[0] == 11:  
                pass
            else:
                print(f"Socket error: {e}")

    def send_chunks(self, conn, CHUNK_SIZE=128): 

        with open(FILE_NAME, "rb") as f:
            self.file_size = os.stat(FILE_NAME)[6]

            http_header = (
                            "HTTP/1.1 200 OK\r\n"
                            "Content-Type: text/csv\r\n"
                            f"Content-Length: {self.file_size}\r\n"
                            "Content-Disposition: attachment; filename=imu-data.csv\r\n"
                            "Connection: close\r\n"
                            "\r\n"
                        ).encode('utf-8')

            conn.sendall(http_header)

            buf = bytearray(CHUNK_SIZE)

            while True:

                self.wtd.feed()

                data = f.readinto(buf)
                
                if data == 0:
                    break   
                conn.sendall(buf[:data]) 

                sleep(0.001)
                  

    def create_socket(self):
        self.onboard_led.value(0)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)

        self.sock.bind((self.ip, self.port))
        self.sock.listen(5)

        self.onboard_led.value(1)
            
if __name__ == "__main__":
    imu_server = None
    try:
        imu_server = IMUServer()

        while True:

            imu_server.wtd.feed()
            if imu_server.collect_flag:

                imu_server.collect_flag = False
                if imu_server.state == 'IDLE':

                    imu_server.turn_off_leds()
                    imu_server.state = 'COLLECTING'
                    print("New dataset button pressed.")

                    imu_server.collect_sensor_data()

                    if not imu_server.sock:
                        imu_server.create_socket()

            imu_server.poll_client()

            gc.collect()

            sleep(0.01)

    except KeyboardInterrupt:
        print("IMU Server stopped.")

    finally:
        
        if imu_server: 
            
            imu_server.led_data_available.value(0)
            imu_server.led_data_being_collected.value(0)

            if imu_server.sock:
                imu_server.sock.close()
                imu_server.onboard_led.value(0)


        
