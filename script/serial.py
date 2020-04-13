import serial
def sendData():
    f = open("../data/coord3D.txt","r")
    line = f.readline()
    print(line)
    f.close()
    port = serial.Serial("COM14", baudrate=115200, timeout=0.1)
    port.write(line.encode())
    port.close()
while(True):
    sendData()