import numpy as np

from filters.kalman_filter import KalmanFilter
from models.simple_model import SimpleModel


if __name__ == '__main__':
    A = np.array([[1]])
    B = np.array([[1]])
    C = np.array([[1]])

    model1 = SimpleModel(A=A, B=B, C=C)

    # generate test observations
    xk = np.array([[1]])
    uk = np.array([[1]])


    observations = [xk]
    for i in range(100 - 1):
        (xk, uk) = model1.get_next_step(xk, uk)
        observations.append(xk)

    observations = np.array(observations)
    # add noise to observations
    observations_with_noise = observations + np.random.normal(0, 3, 100).reshape(100, 1, 1)

    # build Kalman filter
    Q = np.array([[0.1]])
    H = np.array([[1]])
    P = np.array([[1]])
    K = np.array([[2]])
    R = np.array([[2]])
    z = observations_with_noise

    kalm_filter = KalmanFilter(model1, Q, H, P, K, R, z)

    # set x_0 and u_0
    xk = np.array([[1]])
    uk = np.array([[1]])

    res = kalm_filter.do_filtration(50, xk, uk)

    # plot results
    obs_reshaped = observations.reshape(observations.shape[0], )
    obs_wn_reshaped = observations_with_noise.reshape(observations_with_noise.shape[0], )
    res_reshaped = res.reshape(res.shape[0], )


    import matplotlib.pyplot as plt

    t = np.arange(0.0, 50, 1)

    line1, = plt.plot(t, obs_reshaped[:50], 'r', label="Observation with noise")
    line2, = plt.plot(t, obs_wn_reshaped[:50], 'g', label="Ground truth")
    line3, = plt.plot(t, res_reshaped[0:50], 'b', label="Kalman filter")

    legend = plt.legend(handles=[line1, line2, line3], loc=2)
    ax = plt.gca().add_artist(legend)

    plt.show()
