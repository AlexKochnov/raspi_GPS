from serial import Serial

from UBXUtils import POOLMessages

if __name__ == '__main__':
    stream = Serial("/dev/ttyS0", 9600, timeout=1)

    stream.write(POOLMessages.RST)
    # stream.flush()

    for i in range(10):
        print(stream.readline())

    print('Module is reset')