# Runge-Kutta Methods
The implementation of 4th order Runge-Kutta methods using Python.  
NOTE: We focus on how to use rk4 to predict the variation of quaternion with value of gyroscope. If someone is interested in IMU kinetics and how to differentiate quaternion w.r.t. time, please refer to [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508.pdf) in section 5.3 and [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf) in section 1.5.

# TODO List
Add an example, usage, and an explanation jupyter notebook.

# Future
All quaternion operation functions will be wrapped as a standard package in [quaternion_py_tools]().

# Reference
1. [Runge–Kutta methods - Wiki](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods)
2. [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
3. [Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).](https://arxiv.org/pdf/1711.02508.pdf)
4. [Quaternion Derivative](https://ahrs.readthedocs.io/en/latest/filters/angular.html#quaternion-derivative)