import sys
import glob
import serial
import time
import codecs

def serial_ports():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)

            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    print("The serial ports available on the system:")
    print(result)
    return result

def serial_alive(port=None):
    port_status = True
    try:
        s = serial.Serial(port)
        s.close()
    except OSError:
        port_status = False
    return port_status

def bitstring_to_bytes(s):
    v = int(s, 2)
    if v == 0:
        return bytes.fromhex(s)
    b = bytearray()
    while v:
        b.append(v & 0xff)
        v >>= 8
    return bytes(b[::-1])


if __name__ == '__main__':

    class Wrist():
        def __init__(self, port = "COM4"):
            self.port = port
            self.serial_alive(self.port)
            self.switch_on()
            # self.raise_code = "0004"
            # self.stop_code = "0000"
            # self.down_code = "0008"
            self.raise_code = "1010"
            self.down_code = "0101"
            self.stop_code = "0000"
            self.offset = 0

        def switch_on(self):
            self.ser = serial.Serial(self.port, 9600, 8, "N", timeout=50, stopbits=1)
            time.sleep(0.02)

        def serial_alive(self, port=None):
            port_status = True
            try:
                s = serial.Serial(port)
                s.close()
                print(port, "is alive")
            except OSError:
                port_status = False
            return port_status

        def manual(self, value):
            value = bitstring_to_bytes(value)
            self.ser.write(value)

        def wrist_down(self):
            '''

            :return:
            '''
            down_code = bitstring_to_bytes(self.down_code)
            stop_code = bitstring_to_bytes(self.stop_code)
            down_time = 8
            self.ser.write(down_code)
            time.sleep(down_time)
            self.ser.write(stop_code)

        def wrist_up(self, offset = 0, debug = False):
            '''

            :return:
            '''

            raise_code = bitstring_to_bytes(self.raise_code)
            # stop_code = bitstring_to_bytes(self.stop_code)
            self.ser.write(raise_code)
            # raise_time = 3 + offset
            # time.sleep(raise_time)
            # self.ser.write(stop_code)

            # if debug:
            #     start_time = time.time()
            #     self.ser.write(raise_code)
            #     raise_time = 3 + offset
            #     time.sleep(raise_time)
            #     # self.ser.write(stop_code)
            #     end_time = time.time()
            #     timecost = end_time - start_time
            #     print(timecost)

    # Show available serial ports
    # serial_ports()
    wrist = Wrist()

    wrist.wrist_up()

    # a = serial.Serial("COM5", 9600, 8, "N", timeout=50, stopbits=1)
    # code = "1111"
    # code=bitstring_to_bytes(code)
    # a.write(code)
    # table.manual(code)

