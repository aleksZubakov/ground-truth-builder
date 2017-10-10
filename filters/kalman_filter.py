import numpy as np


class KalmanFilter:
    def __init__(self, phys_model, Q, H, P, K, R, z):
        self.__phys_model = phys_model
        self.__Q = Q
        self.__H = H
        self.__P = P
        self.__K = K
        self.__R = R
        self.__z = z

        self.__xk_opt = None
        self.__uk = None

    def __predict_project_error_covariance(self):
        a = self.__phys_model.A
        q = self.__Q
        p = self.__P
        self.__P = a.dot(p).dot(a.T) + q

    def __update_error_covariance(self):
        k = self.__K
        p = self.__P
        h = self.__H
        eye = np.eye(k.shape[0])
        self.__P = (eye - k.dot(h)).dot(p)

    def __compute_kalman_gain(self):
        p = self.__P
        h = self.__H
        r = self.__R
        tmp = (h.dot(p).dot(h.T) + r)
        try:
            tmp = h.dot(p).dot(h.T) + r
            tmp = np.linalg.inv(tmp)
        except np.linalg.linalg.LinAlgError:
            tmp = np.zeros(tmp.shape)

        self.__K = p.dot(h.T).dot(tmp)

    def __compute_xk_opt(self, xk, k):
        self.__xk_opt = xk + self.__K.dot(self.__z[k] - self.__H.dot(xk))

    def do_filtration(self, n_steps, xk_initial, uk_initial):
        self.__xk_opt = xk_initial
        self.__uk = uk_initial

        res = []

        for i in range(n_steps):
            (xk, self.__uk) = self.__phys_model.get_next_step(self.__xk_opt, self.__uk)
            self.__predict_project_error_covariance()
            self.__compute_kalman_gain()
            self.__compute_xk_opt(xk, i)
            self.__update_error_covariance()

            res.append(self.__xk_opt)

        return np.array(res)

    # getters and setters
    @property
    def P(self):
        return self.__P

    # !TODO: write adequate setter
    #     @P.setter
    #     def P(self, P):
    #         self.__P = P

    @property
    def K(self):
        return self.__K

    # !TODO: write adequate setter
    # @z.setter
    #     def z(self, z):
    #         self.__z = z
