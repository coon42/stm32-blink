
import serial

s = serial.Serial("/dev/ttyACM0")

buckets = list(0 for _ in range(512))

def downsample(bin_size):
    avg = 0
    res = []

    for i, val in enumerate(buckets):
        avg += val / bin_size
        if (i+1) % bin_size == 0:
            res += [round(avg / 160)]

    return res


def draw_fft():
    print(chr(27) + "[2J") # clear

    b = downsample(8)

    for i, val in enumerate(b):
        print("#" * val)


for line in s:
    tokens = str(line, "utf8").strip().split(" ")
    b = int(tokens[0])
    val = int(tokens[1])

    buckets[b] = val

    if b == 0:
        draw_fft()

