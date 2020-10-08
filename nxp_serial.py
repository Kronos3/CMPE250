import sys
import threading
import serial

print("Connecting to %s" % sys.argv[0])
ser = serial.Serial(sys.argv[0], timeout=1)


def read_from_port(ser):
    while True:
        sys.stdout.write(ser.read())


thread = threading.Thread(target=read_from_port, args=(ser,))
thread.start()
