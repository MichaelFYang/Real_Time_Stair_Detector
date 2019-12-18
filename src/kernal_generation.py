import numpy as np
import math


def Sigmoid(x):
    return 1 / (1 + math.exp(-x))


if __name__ == "__main__":
    kernal = []
    total_sum = 0
    for i in range(8):
        elem = (i+1) * 0.2
        elem = Sigmoid(elem)
        total_sum += elem
        kernal.append(elem)
    for i in range(8):
        kernal[i] = kernal[i] / total_sum
    print (kernal)

    