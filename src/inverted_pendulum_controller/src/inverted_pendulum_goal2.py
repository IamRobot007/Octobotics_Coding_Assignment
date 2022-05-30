#!/usr/bin/env python3

import rospy
import numpy as np
from numpy import sin, cos, pi
from math import sqrt
import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from inverted_pendulum_sim.msg import ControlForce, CurrentState
from inverted_pendulum_sim.srv import SetParams
from std_msgs.msg import Float32


pend_applied_force = ControlForce()

L = length = 300
m = cart_m = 0.5
M = pendulum_m = 2
dt = 0.05
T_max = 10
t = np.arange(0.0, T_max, dt)
g = 9.8

frequency = 290
amplitude = 120

times = np.arange(0, T_max, dt)
thetas = np.zeros(len(times))
omegas = np.zeros(len(times))

thetas[0] = 0   # initial Theta


def momentum(t):
  w = sqrt(g / L)

  return -0.5 * sin(w * t)


def derivatives(state, t):
  """
  Th'' = - g / L * sin(Th) - M(t) / (m * L^2)
  """
  th, w = state[0], state[1]
  dth = w
  dw = - g / L * sin(th) - momentum(t) / (m * L**2)
  return [dth, dw]


def integrate(state, t, dt):
  """
  Integrate state with Heun's method
  """
  dstate = derivatives(state, t)
  _state = list(map(lambda x: x[0] + x[1] * dt, zip(state, dstate)))
  _dstate = derivatives(_state, t + dt / 2)
  return list(map(lambda x: x[0] + (x[1] + x[2]) * dt / 2, zip(state, dstate, _dstate)))


for i in range(1, len(times)):
  state = [thetas[i - 1], omegas[i - 1]]
  thetas[i], omegas[i] = integrate(state, times[i], dt)


pp.subplot(211)
pp.plot(times, omegas)
pp.plot(times, thetas)
pp.grid(True)

pp.subplot(212)
pp.plot(times, list(map(lambda t: momentum(t), times)))
pp.grid(True)

pp.show()


def set_params_client(theta_initial, x_initial):
    print("waiting for service")
    rospy.wait_for_service('/inverted_pendulum/set_params')
    print("found service")
    try:
        set_params = rospy.ServiceProxy('/inverted_pendulum/set_params', SetParams)
        resp1 = set_params(pendulum_m, length, cart_m, \
                        theta_initial,0,0,\
                        x_initial,0,0)
        print(resp1.success, resp1.message)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def publish_force(input_force):
    pend_applied_force.force = input_force
    pub.publish(pend_applied_force.force)

def frequency_a(data):
    frequency = data
    
    

if __name__ == "__main__":
    rospy.init_node("pid_controller_inverted_pendulum", anonymous=True)
    set_params_client(0,0)
    # set_params_client(0,0)
    pub = rospy.Publisher("/inverted_pendulum/control_force", ControlForce, queue_size=10)
    sub = rospy.Subscriber("/inverted_pendulum/sin_force_frequency", Float32, frequency_a)
    rate = rospy.Rate(100)
    start_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        t = start_time - curr_time
        f  = amplitude*sin(2*pi*frequency*t)
        publish_force(f)
        rate.sleep()