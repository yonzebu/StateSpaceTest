import math
from utilities.state_space.state_space_utils import *
from utilities.state_space.state_space_gains import StateSpaceGains, GainsList
from utilities.motor import MotorType
from utilities.state_space.ss_sim import StateSpaceControlSim


# This is a theoretical state space model for a BAG with velocity control
# Adding position control, however, would be trivial
def create_gains():

    # Motor constants
    free_speed, free_current, stall_torque, stall_current, battery_voltage = MotorType._PRO775.value

    # torque / Kt = I-stall, so Kt = torque / I-stall in N-m / A
    Kt = stall_torque / stall_current
    # V-battery = I-stall * R, so R = V-battery / I-stall
    R = battery_voltage / stall_current
    # V-battery = I-free * R + w-free / Kv, so Kv = w-free / (V-battery - I-free * R)
    Kv = free_speed / (battery_voltage - free_current * R)
    # Damping coefficient, determines torque caused by given speed, sort of
    # Probably not using this, actually
    # Although I'm using it right now I think
    d = free_current * Kt / free_speed

    # Constants for the system the motor is used in
    # Gear ratio (torque-out / torque-in)
    GR = 3.
    # Moment of inertia in kg-m^2, assumed 1 for simplicity
    MoI = .004
    # Efficiency of the system is the ratio between actual output torque and expected output torque
    efficiency = 1.

    # k1 and k2, which determine the A and B matrices, are determined by solving the motor characterization equation
    # for angular acceleration
    k1 = -efficiency * GR * GR * ((Kt / (Kv * R * MoI)) + (d / MoI))
    k2 = efficiency * Kt * GR / (R * MoI)

    # Sensor ratio for CTRE Magnetic Encoders with Talon SRX's is 4096 ticks/rotation
    # Angular velocity is measured in ticks / .1 s, so the sensor ratio must be adjusted
    # Sensor ratio converts internal state (rad/s) to sensor units (ticks / .1s)
    sensor_ratio = 4096. / (2. * math.pi * 10.)
    # Sensor ratio for position doesn't have deciseconds, so no 10
    pos_sensor_ratio = 4096 / (2. * math.pi)

    # Setting up the system based on constants solved for via motor characterization
    A = np.asmatrix([
        [0., 1.],
        [0., k1]
    ])

    B = np.asmatrix([
        [0],
        [k2]
    ])

    C = np.asmatrix([
        [pos_sensor_ratio, 0]
    ])

    D = np.zeros((1, 1))

    # These values were kind of arbitrary, I should probably check the accuracy of sensors, and try to find some way
    # to maybe determine how much disturbance noise to expect
    Q_noise = np.asmatrix([
        [0, 0],
        [0, 1.e-2]
    ])

    R_noise = np.asmatrix([
        [1.e-3]
    ])

    dt = .02

    A_d, B_d, Q_d, R_d = c2d(A, B, Q_noise, R_noise, dt)

    # LQR weight matrix Q, a diagonal matrix whose diagonals express how bad it is for the corresponding state variable
    # to be in the wrong place.
    # I found a thing that said to weight LQR weight matrices so that they are diagonal,
    # and to use 1 / (acceptable error)^2 for each diagonal entry, each of which correspond to one state variable.
    # In this case, I decided acceptable velocity error was .01 rad/s and acceptable position error was .01 rad, so
    # the entries in Q_weight are calculated accordingly.
    p = 0.1
    Q_weight = np.asmatrix([
        [(p / 1.e-2)**2, 0],
        [0, (p / 1.e-1)**2]
    ])

    # LQR weight matrix R, a diagonal matrix similar to Q, except with regards to the inputs, rather than states
    # Higher values along the diagonals place higher constraint on corresponding inputs.
    # The thing that said to weight Q matrices said to weight R matrices in the same way, so, since acceptable max input
    # is battery voltage (limited slightly in this case in case of mechanical inefficiency), the entry in R_weight is
    # calculated accordingly
    R_weight = np.asmatrix([
        [1. / ((battery_voltage * 0.5/6.) ** 2)]
    ])

    # This was an arbitrary choice, and I'm going to actually have to look into optimal pole placement and such
    # Maybe also matlab/octave state space sim stuff
    # Pole placement actually doesn't seem to quite be working for velocity-controlled motors
    desired_poles = [.5 - 0.1j, .5 + 0.1j]

    # Pole placement
    # K_d = place_poles(A_d, B_d, desired_poles)
    # K_d = np.asmatrix([[10.]])
    K_d = dlqr(A_d, B_d, Q_weight, R_weight)
    # print(np.linalg.eigvals(A_d - (B_d * K_d)))

    # Kalman gains, optimal matrix for estimating and stuff
    L_d = discrete_kalman(A_d, C, Q_d, R_d)

    # print(L_d)

    # Feedforward matrix
    Kff = np.asmatrix(feedforward_gains(B_d))

    # Reference-tracking matrix, used to track arbitrary step reference measurements
    # Calculated in discrete time as N = inv( -C_d * inv(A_d - B_d*K - I) * B_d), where I is the identity matrix
    # equivalent in dimension to A
    n = A_d.shape[0]
    N = np.asmatrix(-np.linalg.inv(-C * np.linalg.inv(A_d - B_d*K_d - np.identity(n)) * B_d))

    u_max = np.asmatrix([
        [battery_voltage / 12.]
    ])
    u_min = -u_max

    gains = GainsList(StateSpaceGains('MotorGains', A_d, B_d, C, D, Q_d, R_d, K_d, L_d, Kff, N, u_min, u_max, dt))

    return gains, u_max, u_min


def reference_calculator(time: float):
    if time < 4:
        return np.zeros((2, 1))
    elif time < 8:
        return np.asmatrix(
            [[3.14],
            [0.]]
            )
    else:
        return np.asmatrix(
            [[-3.14],
            [0.]]
            )


def voltage_calculator(time: float):
    return np.zeros((1, 1)) if time < 0. else np.asmatrix([[12.]])


def sim():
    gains_list, u_max, u_min = create_gains()
    gains = gains_list.get_gains(0)
    x_initial = np.asmatrix([
        [-3.14],
        [0.]
    ])
    x_hat_initial = x_initial
    u_initial = np.zeros((1, 1))

    sim = StateSpaceControlSim(gains, x_hat_initial=x_hat_initial, u_initial=u_initial, x_initial=x_initial,
                               u_max=u_max, u_min=u_min)

    # Options: theta, theta_dot, u, y (angle), theta_hat, theta_hat_dot
    # Currently selected: theta, theta_hat
    plot_settings = (True, False, False, False, True, False)
    duration = 10.

    sim.plot_reference_tracking(duration=duration, plot_settings=plot_settings, reference_calculator=reference_calculator)


if __name__ == '__main__':
    sim()

