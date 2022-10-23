
# step size
h       = (xn     -     x0) /  n
w_alpha = (w_hat2 - w_hat1) / dt
dt      = (w_hat2 - w_hat1) / w_alpha

# init
x0
w_hat1

y0
dq_0

xn
w_hat2

yn
dq

# k1
k1 = h * f(x0, y0)
k1_q = 0.5 * Omega(w_hat) * dq_0 * dt

# k2
x0+h/2  === 0.5 * Omega(w_hat1 + 0.5 * w_alpha * dt)
y0+k1/2 === quatnorm(dq_0 + 0.5 * k1_q)
k2 = h * f((x0+h/2), (y0+k1/2))
k2_q = 0.5 * Omega(w_hat1 + 0.5 * w_alpha * dt) * quatnorm(dq_0 + 0.5 * k1_q) * dt

# k3
k3 = h * f((x0+h/2), (y0+k2/2))
k3_q = 0.5 * Omega(w_hat1 + 0.5 * w_alpha * dt) * quatnorm(dq_0 + 0.5 * k2_q) * dt

# k4
k4 = h * f((x0+h), (y0+k3))
k4_q = 0.5 * Omega(w_hat1 + 0.5 w_alpha * dt) * quatnorm(dq_0 + k3_q) * dt

yn    =         y0 + (               k1 +           2 *   k2 +           2 *   k3 +                 k4) / 6
dq = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q)


new_q = quat_multiply(dq, q_0)

# differentiate y with respect to t
# By IMU kinematics, we define quaternion time derivative as section 1.5 in <Indirect Kalman Filter for 3D Attitude Estimation>
# This is the quaternion time derivative not the delta quaternion time derivative;
def f(x, y):
    0.5 * Omega(x) * y

# Here define the general form of rk4.
# Use rk4 to predict quaternion: Quaternion kinematics for the error-state Kalman filter, p.74
# step size : h === dt
# derivative: dy/dx === f(x, y)
# y0        : dq0
# x0        : gyr0
# xn        : gyr1
def rk4(x0, y0, xn, h):
    # NOTE: to time, our step size is dt;
    #       to gyro, our step size is dgyr = (gyr1 - gyr0) / dt
    k1 = h * f(x0, y0)
    k2 = h * f(x0 + 0.5 * h, y0 + 0.5 * k1)
    k3 = h * f(x0 + 0.5 * h, y0 + 0.5 * k2)
    k4 = h * f(x0 + h, y + k3)
    yn = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return yn

