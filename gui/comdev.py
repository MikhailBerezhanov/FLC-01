import serial
import serial.tools.list_ports

CR = serial.CR
LF = serial.LF

PARITY_NONE = serial.PARITY_NONE
STOPBITS_ONE = serial.STOPBITS_ONE
EIGHTBITS = serial.EIGHTBITS

# Get list of available COM ports in system now 
def get_avail_ports():
    ports = serial.tools.list_ports.comports()
    port_tuple = {}
    for p in ports:
        port_tuple[p.device] = p.description
    return port_tuple

class OpenPortException(Exception): pass

class Comdevice():
    def __init__(self, port_name, baud, parity, stopbits, bytesize, timeout):
        self.opened = False

        try:
            self.port = serial.Serial(
                port=port_name,
                baudrate=baud,
                parity=parity,
                stopbits=stopbits,
                bytesize=bytesize,
                timeout=timeout)

            if self.port.isOpen:
                print("Connected to: " + self.port.portstr)
                self.opened = True

        except serial.SerialException:
            print("Unable to open port '" + port_name + "'")
            # Push exception further
            if __name__ != "__main__": raise OpenPortException()

        except ValueError:
            print("Port init ValueError")
            self.opened = False
            self.port.close()

    def write(self, byte_arr):
        if self.opened:
            try:
                self.port.write(byte_arr)
            except serial.SerialException:
                print("port.write SerialException")
            except serial.SerialTimeoutException:
                print("port.write SerialTimeoutException")
            except Exception:
                print("port.write Unknown exception")
        else:
            print("Write failed, Port is closed!")

    def read_until(self, symb, max_size):
        if self.opened:
            try:
                return self.port.read_until(symb, size=max_size)
            except serial.SerialException:
                print("port.read_until SerialException")
            except serial.SerialTimeoutException:
                print("port.read_until SerialTimeoutException")
            except Exception:
                print("port.read_until Unknown exception")
        else:
            print("Read_until failed, Port is closed!")

    def close(self):
        if self.opened:
            self.port.close()
            self.opened = False



            
