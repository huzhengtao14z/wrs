import serial
import time
class Sender:
    TERMINATOR = '\r'.encode("UTF8")

    def __init__(self, device = 'COM3',baud = 9600, timeout = 1):
        self.serial = serial.Serial(device, baud, timeout = timeout)

    def receive(self):
        line = self.serial.read_until(self.TERMINATOR)
        return line.decode('UTF8').strip()

    def send(self, text):
        line = '%s\r\f' %text
        self.serial.write(line.encode('UTF8'))

        return text == self.receive()

    def close(self):
        self.serial.close()

if __name__ == "__main__":
    s = Sender()
    # s.send("wristup")
    print("check")
    while True:
        s.send("wristup()")
        time.sleep(1)
        s.send("wristdown()")
        time.sleep(1)