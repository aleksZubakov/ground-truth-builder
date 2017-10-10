import numpy as np

from filters.kalman_filter import KalmanFilter
from models.simple_model import SimpleModel


if __name__ == '__main__':
    A = np.array([[0.3, 1.1], [0.9, 0]])
    B = np.array([[0], [0]])
    C = np.array([[0]])

    model2 = SimpleModel(A=A, B=B, C=C)

    # generate test observations
    xk = np.array([[1], [0.3]])
    uk = np.array([[0]])

    observations = [xk]
    for i in range(100 - 1):
        (xk, uk) = model2.get_next_step(xk, uk)
        observations.append(xk)

    observations = np.array(observations)
    cov = np.array([[1, 0],
                    [0, 2]])
    observations_with_noise = observations + np.random.multivariate_normal([0, 0], cov, 100).reshape(100, 2, 1)

    #build Kalman Filter
    R = np.array([[1, 0],
                  [0, 1.7]])
    H = np.array([[1, 0],
                  [0, 1]])
    P = np.array([[1, 0],
                  [0, 1]])
    K = np.array([[1, 0],
                  [0, 1]])
    Q = np.array([[1, 0],
                  [0, 1.1]])
    z = observations_with_noise

    kalm_filter = KalmanFilter(model2, Q, H, P, K, R, z)


    # set x_0 and u_0
    xk = np.array([[1], [0.3]])
    uk = np.array([[0]])

    res = kalm_filter.do_filtration(30, xk, uk)

    # plot results
    import matplotlib.pyplot as plt

    line1, = plt.plot(observations[:20, 1], observations[:20, 0], 'go-',
                      label="Ground truth")
    line2, = plt.plot(observations_with_noise[:20, 1], observations_with_noise[:20, 0], 'ro-',
                      label="Observations with noise")
    line3, = plt.plot(res[0:20, 1], res[0:20, 0], 'bo-',
                      label="Kalman filter")

    legend = plt.legend(handles=[line1, line2, line3], loc=2)

    ax = plt.gca().add_artist(legend)

    plt.show()
