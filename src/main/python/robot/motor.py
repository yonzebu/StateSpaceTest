import numpy as np
import scipy, math
from utilities.state_space_utils import c2d, dlqr, discrete_kalman
from utilities.state_space_gains import StateSpaceGains, GainsList


# This is a theoretical state space model for a 775pro with velocity control
# Adding position control, however, would be trivial
def create_gains():

    # Motor constants

    # RPM to rad/s, final unit is rad/s
    free_speed = 18730. * 2. * math.pi / 60.
    # Current in Amps
    free_current = .7
    # Stall Torque in N-m
    stall_torque = .71
    # Stall current in Amps
    stall_current = 134.
    # Battery voltage in Volts
    battery_voltage = 12.

    # torque / Kt = I-stall, so Kt = torque / I-stall in N-m / A
    Kt = stall_torque / stall_current
    # V-battery = I-stall * R, so R = V-battery / I-stall
    R = battery_voltage / stall_current
    # V-battery = I-free * R + w-free / Kv, so Kv = w-free / (V-battery - I-free * R)
    Kv = free_speed / (battery_voltage - free_current * R)
    # Damping coefficient, determines torque caused by given speed, sort of
    # Probably not using this, actually
    d = stall_current * Kt / free_speed

    # Constants for the motor's system

    # Gear ratio (torque-out / torque-in), assumed to be one for simplicity
    GR = 1
    # Moment of inertia in kg-m^2, assumed 1 for simplicity
    MoI = 1

    # k1 and k2, which determine the A and B matrices, are determined by solving the motor characterization equation
    # for angular acceleration
    k1 = GR * Kt / (Kv * R * MoI)
    k2 = Kt * GR / (R * MoI)

    # Sensor ratio for CTRE Magnetic Encoders with Talon SRX's is 4096 ticks/rotation
    # Angular velocity is measured in ticks / .1 s, so the sensor ratio must be adjusted
    # Sensor ratio converts internal state (rad/s) to sensor units (ticks / .1s)
    sensor_ratio = 10. * 2. * math.pi / 4096

    # Setting up the system based on constants solved for via motor characterization
    A = np.asmatrix([
        [0, 1],
        [0, k1]
    ])

    B = np.asmatrix([
        [0],
        [k2]
    ])

    C = np.asmatrix([
        [sensor_ratio, 0]
    ])

    D = np.zeros((1, 1))

    # These values were kind of arbitrary, I should probably check the accuracy of sensors, and try to find some way
    # to maybe determine how much disturbance noise to expect
    Q_noise = np.asmatrix([
        [1e-3, 0],
        [0, 1e-4]
    ])

    R_noise = np.asmatrix([
        [1e-3]
    ])

    dt = .01

    A_d, B_d, Q_d, R_d = c2d(A, B, Q_noise, R_noise, dt)

    # LQR weight matrix Q, a diagonal matrix whose diagonals express how bad it is for the corresponding state variable
    # to be in the wrong place. There is only one state variable here, and it doesn't need a very large penalty, since
    # when only speed is being controlled, it's rarely very important
    Q_weight = np.asmatrix([
        [10, 0],
        [0, .3]
    ])

    # LQR weight matrix R, a diagonal matrix similar to Q, except with regards to the inputs, rather than states
    # Higher values along the diagonals place higher constraint on corresponding inputs
    # There is only one input here, and it is limited in how large it can be, so the weighting is chosen to be larger
    # than in the Q matrix
    R_weight = np.asmatrix([
        [10]
    ])

    # LQR
    K = dlqr(A_d, B_d, Q_weight, R_weight)

    L = discrete_kalman(A_d, C, Q_noise, R_noise)

    gains = GainsList(StateSpaceGains(A_d, B_d, C, D, Q_d, R_d, K, L, dt, 'MotorGains'))
    gains.get_gains(0).print_gains()

    return gains