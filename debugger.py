import serial

class Debugger:
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, 115200)

    def read(self):
        return self.ser.readline()

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()

if __name__ == '__main__':
    debugger = Debugger('COM5')
    
    while True:
        print(debugger.read())
        print("\n")