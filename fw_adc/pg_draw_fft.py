
#
# Draw FFT using pygame
#

import pygame
import serial


def draw_bucket(display, i, v):
    x = v
    y = 1024 * (i/512.0);
    rect = (10, y, x, 1)

    pygame.draw.rect(display, (255.0, 255.0, 255.0), rect)


def draw_total(display, buckets):
    total = sum(buckets[10:]) - len(buckets)

    y = total

    rect = (1020, 0, 1, y)
    pygame.draw.rect(display, (255.0, 0, 0), rect)


def draw_buckets(display, buckets):
    """Draw buckets"""
    for i, v in enumerate(buckets):
        draw_bucket(display, i, v)

    # Calculate total energy
    draw_total(display, buckets)

    draw_downsampled(display, buckets)

    pygame.display.update()


def draw_downsampled(display, buckets):
    dbuckets = []
    for i in range(2, len(buckets), 8):
        dbuckets += [sum(buckets[i:i+8])]

    for i, v in enumerate(dbuckets):
        y = v
        rect = (100 + i*10, 1020, 8, -y)
        pygame.draw.rect(display, (0, 0, 255.0), rect)


def receive(port, display):
    s = serial.Serial(port)

    buckets = [0 for _ in range(512)]

    for line in s:
        t = str(line, "utf8").strip().split(" ")
        b = int(t[0])
        v = int(t[1])
        buckets[b] = v

        if b == 0:
            display.fill((0,0,0))
            draw_buckets(display, buckets)


def main():
    # init pygame
    pygame.init()
    display = pygame.display.set_mode((1024, 1024), 0, 32)

    receive("/dev/ttyACM0", display)


if __name__ == "__main__":
    main()
