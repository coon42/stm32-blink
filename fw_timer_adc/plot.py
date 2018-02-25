
import serial

s = serial.Serial("/dev/ttyACM0")

for line in s:
    tokens = str(line, "utf8").strip().split(" ")
    val = int(tokens[1])
    print((val) * "#")

