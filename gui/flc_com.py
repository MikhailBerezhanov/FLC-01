from comdev import *
import json
import sys

# Device with serial interface (fitolamp)
class Fitolamp(Comdevice):
    def __init__(self, port_name):
        self.status = {"result_code": -1,
                       "result_text": "NoData",
                       "current_dtime": "NoData",
                       "power_state": -1,
                       "ch1_power": -1,
                       "ch2_power": -1,
                       "ch3_power": -1,
                       "start_time": "NoData",
                       "stop_time": "NoData",
                       "smooth_control": False
                       }

        try:
            Comdevice.__init__(self, port_name,
                               baud=115200,
                               parity=PARITY_NONE,
                               stopbits=STOPBITS_ONE,
                               bytesize=EIGHTBITS,
                               timeout=1)
        except OpenPortException:
            raise
            
    def print_status(self):
        for item in self.status:
            print(item + ":\t" + str(self.status[item]))
            
    # private function
    def __send_request(self, request):
        self.write(request.encode('utf-8'))
        responce = self.read_until(CR+LF, 512)
                
        result = responce.decode("utf-8")
        try:
            parsed = json.loads(result)
        except json.JSONDecodeError:
            print("JSONDecodeError")
            print(sys.exc_info()[:2])
            return False
        else:
            # update settings values
            for item in self.status:
                self.status[item] = parsed[item]
            if parsed["result_code"] == 0 and parsed["result_text"] == "OK" :
                return True
            else:
                return False

    def get_status(self):
        request = '{}\r\n'
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_start_time(self, value):
        request = json.dumps({"start_time":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_stop_time(self, value):
        request = json.dumps({"stop_time":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_current_time(self, value):
        request = json.dumps({"current_time":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_red_power(self, value):
        request = json.dumps({"ch2_power":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_blue_power(self, value):
        request = json.dumps({"ch3_power":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_white_power(self, value):
        request = json.dumps({"ch1_power":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_smooth_control(self, value):
        request = json.dumps({"smooth_control":str(value)}) + "\r\n"
        return "OK" if self.__send_request(request) else self.status["result_text"]

    def set_all(self): pass

    def close(self):
        Comdevice.close(self)

# Self-Test
if __name__ == "__main__":
    for port in get_avail_ports():
        print(port)
    fitolamp = Fitolamp("COM8")
    fitolamp.get_status()
    print("Set current time:\t" + fitolamp.set_current_time("17:00"))
    print("Set start time:\t" + fitolamp.set_start_time("18:00"))
    fitolamp.print_status()
    fitolamp.close()
    print("Test finished, Port closed")

    
