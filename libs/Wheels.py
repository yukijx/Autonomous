import threading
from time import sleep, time
from libs.utilities import send_udp

class WheelInterface:
    def __init__(self) -> None:
        self._START_BYTE = 0x23
        self._MESSAGE_TYPE = 0x00
        self._SEND_PERIOD = 0.1
        self._TIMEOUT = 3.0
        self._running = False
        self._speeds = [0, 0]
        self._last_received = time()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._update_loop, name=(
            'update wheel speeds'), args=())


    def start(self, ip, port):
        self._ip = ip
        self._port = port
        if not self._running:
            self._running = True
            self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()

    def set_wheel_speeds(self, left, right):
        self._last_received = time()
        with self._lock:
            self._speeds = [left, right]
    
    def get_wheel_speeds(self):
        with self._lock:
            return self._speeds

    def _update_loop(self):
        print('this is the real one')
        while self._running:
            with self._lock:
                self._send_wheel_speeds()
            sleep(self._SEND_PERIOD)

    def _send_wheel_speeds(self):
        """
        Sends wheel speed message to the rover
        """
        left, right = self._speeds

        # send 0 if set has not been called in a while
        if time() - self._last_received > self._TIMEOUT:
            # set self variable and function variables to 0
            self._speeds = left, right = [0, 0]

        #check if valid
        if left < -1 or left > 1:
            raise ValueError("Left wheel speed out of range")
        if right < -1 or right > 1:
            raise ValueError("Right wheel speed out of range")
        
        #create message
        msg = bytearray(9)

        #add start byte and message type
        msg[0] = self._START_BYTE
        msg[1] = self._MESSAGE_TYPE

        #convert to 0-252
        speed_fix = []
        for i in range(6):
            if i < 3:
                x = (left+1) * 126
            else:
                x = (right+1) * 126
            speed_fix.append(int(x))

        # honestly the checksum is absolutely redundant and should be removed everywhere
        cs = 0
        for i in range(6):
            msg[i + 2] = speed_fix[i]
            cs += speed_fix[i]

            #add verification bit
            msg[8] = cs&0xff #capped at 8 binary characters of length

        #send wheel speeds
        send_udp(self._ip, self._port, msg)

class MockedWheelInterface(WheelInterface):
    def _update_loop(self):
        while self._running:
            # skip the wheel speed update
            # print('Wheel speed in wheel thread: ', self._speeds)
            with self._lock:
                self._send_wheel_speeds()
            sleep(self._SEND_PERIOD)