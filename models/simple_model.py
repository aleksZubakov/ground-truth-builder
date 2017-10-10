import numpy as np


class SimpleModel:
    def __init__(self, A=None, B=None, C=None):
        self.__A = A
        self.__B = B
        self.__C = C

    def get_next_step(self, x_k, u_k):
        return self.__A.dot(x_k) + self.__B.dot(u_k), self.__C.dot(u_k)

    # getters and setters
    @property
    def A(self):
        return self.__A

    @property
    def B(self):
        return self.__B

    @property
    def C(self):
        return self.__C
