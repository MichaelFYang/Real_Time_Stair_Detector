import numpy as np
import math


def Sigmoid(x):
    return 1 / (1 + math.exp(-x))

def ReLu(x):
    if x>0:
        return x
    else:
        return 0


if __name__ == "__main__":
    kernal = []
    total_sum = 0
    for i in range(8):
        elem = i * 0.5
        elem = ReLu(elem)
        total_sum += elem*elem
        kernal.append(elem)
    for i in range(8):
        kernal[i] = kernal[i] / np.sqrt(total_sum)
    print (kernal)

    