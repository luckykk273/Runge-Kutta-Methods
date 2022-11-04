from quat_utils import *


def rk4(w0, w1, dt):
    """
    Given initial gyro value and final gyro value, predict delta quaternion.
    
    See Also
    --------

    step size:
        1. time step size h = dt
        2. gyro step size dw = (w1 - w0) / dt

    derivative function: dy/dx = f(x, y)

    initial value: t0 = w0, y0 = dq0

    approximated value: tn = w1, yn = dqn
    
    :param w0: initial gyro value
    :param w1: final gyro value
    :param dt: duration time(as time step size h in RK4 definition)
    :return: predicted delta quaternion given w0 and w1
    """
    def f(x, y):
        return 0.5 * np.matmul(omega(x), y)

    # pre-compute things
    dw = (w1 - w0) / dt
    # y0
    dq0 = np.array([1.0, 0.0, 0.0, 0.0])
    # k1 = h * f(x0, y0)
    k1 = dt * f(w0, dq0)
    # k2 = h * f(x0 + h/2, y0 + k1/2)
    k2 = dt * f(w0 + 0.5 * dw * dt, quat_norm(dq0 + 0.5 * k1))
    # k3 = h * f(x0 + h/2, y0 + k2/2)
    k3 = dt * f(w0 + 0.5 * dw * dt, quat_norm(dq0 + 0.5 * k2))
    # k4 = h * f(x0 + h, y0 + k3)
    k4 = dt * f(w0 + dw * dt, quat_norm(dq0 + k3))
    # yn = y0 + (k1 + 2*k2 + 2*k3 + 2*k4) / 6
    dqn = quat_norm(dq0 + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0)

    return dqn
