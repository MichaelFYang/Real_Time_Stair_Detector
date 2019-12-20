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
    kernel_size = 5
    total_sum = 0
    for i in range(kernel_size):
        elem = (i+1) * 0.5
        elem = Sigmoid(elem)
        total_sum += elem*elem
        kernal.append(elem)
    for i in range(kernel_size):
        kernal[i] = kernal[i] / np.sqrt(total_sum)
    print (kernal)

    