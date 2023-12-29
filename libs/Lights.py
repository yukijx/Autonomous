import threading
from time import sleep
from libs.utilities import send_udp

class Lights:
    def __init__(self):
        self._START_BYTE = 0x23
        self._MESSAGE_TYPE = 0x02
        self._found_event = threading.Event()
        self._running = False
        self.current_color = 'o'
        self._thread = threading.Thread(target=self._update_lights, name="Update Lights", args=(self._found_event,))

    def _update_lights(self, event: threading.Event) -> None:
        while self._running:
            if event.is_set():
                self.current_color = 'g'
                self._send_led_message(self._host, self._port, 'g')
                sleep(.5)
                self.current_color = 'o'
                self._send_led_message(self._host, self._port, 'o')
                sleep(.5)
            else:
                self.current_color = 'r'
                self._send_led_message(self._host, self._port, 'r')
                sleep(1)

    def start(self, host: str, port: int) -> None:
        """
        Starts the lights thread
        
        Args:
            host (str): IP address of rover
            port (int): Port of rover
        """
        self._host = host
        self._port = port
        self._running = True
        self._thread.start()

    def found(self) -> None:
        self._found_event.set()

    def not_found(self) -> None:
        self._found_event.clear()

    def stop(self) -> None:
        self.not_found()
        if self._running:
            self._running = False
            self._thread.join()

    def _send_led_message(self, host: str, port: int, color: str) -> None:
        """
        Sends LED message to the rover

        Args:
            host (str): IP address of rover
            port (int): Port of rover
            color (str): Color of LED ('r', 'g', 'b', 'o'(off))
        """
        red = 0
        green = 0
        blue = 0
        msg = bytearray(5)
        msg[0] = self._START_BYTE
        msg[1] = self._MESSAGE_TYPE
        if color == 'r':
            red = 255
        elif color == 'g':
            green = 255
        elif color == 'b':
            blue = 255
        msg[2] = red
        msg[3] = green
        msg[4] = blue
        send_udp(host,port,msg)