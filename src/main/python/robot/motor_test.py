import numpy as np
import math
from utilities.state_space_utils import c2d, dlqr, discrete_kalman, feedforward_gains, place_poles
from utilities.state_space_gains import StateSpaceGains, GainsList
from utilities.motor import MotorType


# This is a theoretical state space model for a 775pro with velocity control
# Adding position control, however, would be trivial
def create_gains():

    # Motor constants
    free_speed, free_current, stall_torque, stall_current, battery_voltage = MotorType._775PRO.value

    # torque / Kt = I-stall, so Kt = torque / I-stall in N-m / A
    Kt = stall_torque / stall_current
    # V-battery = I-stall * R, so R = V-battery / I-stall
    R = battery_voltage / stall_current
    # V-battery = I-free * R + w-free / Kv, so Kv = w-free / (V-battery - I-free * R)
    Kv = free_speed / (battery_voltage - free_current * R)
    # Damping coefficient, determines torque caused by given speed, sort of
    # Probably not using this, actually
    d = stall_current * Kt / free_speed

    # Constants for the system the motor is used in
    # Gear ratio (torque-out / torque-in), assumed to be one for simplicity
    GR = 1.
    # Moment of inertia in kg-m^2, assumed 1 for simplicity
    MoI = 1.
    # Efficiency of the system is the ratio between actual output torque and expected output torque
    efficiency = .9

    # k1 and k2, which determine the A and B matrices, are determined by solving the motor characterization equation
    # for angular acceleration
    k1 = GR * Kt / (Kv * R * MoI)
    k2 = Kt * GR / (R * MoI)

    # Sensor ratio for CTRE Magnetic Encoders with Talon SRX's is 4096 ticks/rotation
    # Angular velocity is measured in ticks / .1 s, so the sensor ratio must be adjusted
    # Sensor ratio converts internal state (rad/s) to sensor units (ticks / .1s)
    sensor_ratio = 4096. / (2. * math.pi * 10.)

    # Setting up the system based on constants solved for via motor characterization
    A = np.asmatrix([
        [k1]
    ])

    B = np.asmatrix([
        [k2]
    ])

    C = np.asmatrix([
        [sensor_ratio]
    ])

    D = np.zeros((1, 1))

    # These values were kind of arbitrary, I should probably check the accuracy of sensors, and try to find some way
    # to maybe determine how much disturbance noise to expect
    Q_noise = np.asmatrix([
        [0]
    ])

    R_noise = np.asmatrix([
        [1e-3]
    ])

    dt = .01

    A_d, B_d, Q_d, R_d = c2d(A, B, Q_noise, R_noise, dt)

    # LQR weight matrix Q, a diagonal matrix whose diagonals express how bad it is for the corresponding state variable
    # to be in the wrong place.
    # I found a thing that said to weight LQR weight matrices so that they are diagonal,
    # and to use 1 / (acceptable error)^2 for each diagonal entry, each of which correspond to one state variable.
    # In this case, I decided acceptable error was 15 deg/s, or pi/12 rad/s, so the entry in Q_weight is calculated
    # accordingly.
    Q_weight = np.asmatrix([
        [(12. / math.pi)**2]
    ])

    # LQR weight matrix R, a diagonal matrix similar to Q, except with regards to the inputs, rather than states
    # Higher values along the diagonals place higher constraint on corresponding inputs.
    # The thing that said to weight Q matrices said to weight R matrices in the same way, so, since acceptable max input
    # is battery voltage (limited slightly in this case in case of mechanical inefficiency), the entry in R_weight is
    # calculated accordingly
    R_weight = np.asmatrix([
        [1. / ((battery_voltage * 5./6.) ** 2)]
    ])

    desired_poles = [-10.]

    # LQR
    K = place_poles(A_d, B_d, desired_poles)

    # Kalman gains, optimal matrix for estimating and stuff
    L = discrete_kalman(A_d, C, Q_noise, R_noise)

    Kff = np.asmatrix(feedforward_gains(B_d))

    u_min = np.asmatrix([
        [-10.]
    ])
    u_max = - u_min

    gains = GainsList(StateSpaceGains(A_d, B_d, C, D, Q_d, R_d, K, L, Kff, u_min, u_max, dt, 'MotorGains'))

    return gains