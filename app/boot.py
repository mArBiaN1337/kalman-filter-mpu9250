import machine 
import esp
import gc
import micropython
# pyright: reportMissingImports=false

class Boot:
    
    def __init__(self):

        micropython.alloc_emergency_exception_buf(100)
        esp.osdebug(None)
        gc.collect()

        self.btn1 = machine.Pin(25, machine.Pin.IN, machine.Pin.PULL_UP)
        self.onboard_led = machine.Pin(2, machine.Pin.OUT)
        self.led1 = machine.Pin(13, machine.Pin.OUT)
        self.led2 = machine.Pin(18, machine.Pin.OUT)
        

        self.turn_off_leds()

    def turn_off_leds(self):
        self.onboard_led.value(0)
        self.led1.value(0)
        self.led2.value(0)

if __name__ == "__main__":
    pass