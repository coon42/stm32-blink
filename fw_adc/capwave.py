
import time

import serial
import numpy as np

from scipy.io import wavfile


def receive(port, filename):
    s = serial.Serial(port)

    tlen = 50
    ttotal = tlen * 1024
    buf = np.zeros(shape=(ttotal,), dtype="int16")

    t0 = time.time()
    i = 0
    for line in s:
        t = str(line, "utf8").strip().split(" ")
        v = int(t[1])
        buf[i] = v

        i += 1
        if i == ttotal:
            break

    # remove dc offset
    buf = buf - np.average(buf)

    # amplify
    buf = buf * 255

    t1 = time.time()

    srate = round(ttotal / (t1 - t0))
    print("rate: {} samples/sec".format(srate))

    # write buffer
    wavfile.write(filename, srate, buf.astype("int16"))


if __name__ == "__main__":
    receive("/dev/ttyACM0", "foo.wav")

