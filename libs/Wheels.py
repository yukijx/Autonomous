import threading
from time import sleep
from libs.utilities import send_udp

class WheelInterface:
    def __init__(self) -> None:
        self._START_BYTE = 0x23
        self._MESSAGE_TYPE = 0x00
        self._SEND_PERIOD = 0.1
        self._speeds = [0, 0]

    def start(self, ip, port):
        self._ip = ip
        self._port = port
        self._running = True
        self._thread = threading.Thread(target=self._update_loop, name=(
            'update wheel speeds'), args=())
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()

    def set_wheel_speeds(self, left, right):
        self._speeds = [left, right]

    def _update_loop(self):
        while self._running:
            self.send_wheel_speeds(self._speeds[0], self._speeds[1])
            sleep(self._SEND_PERIOD)

    def send_wheel_speeds(self, left, right):
        """
        Sends wheel speeds to the rover
        left: -1 to 1
        right: -1 to 1
        """

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
            # self._speeds = speeds
            sleep(self._SEND_PERIOD)