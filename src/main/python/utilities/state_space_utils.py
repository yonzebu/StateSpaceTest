import numpy as np
import scipy
import scipy.signal

""" 
A bunch of helper functions for dealing with state space stuffs 
Mostly a wrapper to various scipy routines because scipy is majik.
Has: pole placement, LQR, Kalman gains, controllability/observability matrices, continuous to discrete conversion

Heavily inspired by 1678's controls helper stuff
"""


def check_validity(A=None, B=None, C=None, D=None, Q_noise=None, R_noise=None, K=None, L=None, Kff=None, N=None):
    """Checks the validity of the system based on the sizes of matrices in the system"""

    if A is not None:
        A = np.asmatrix(A)
    if B is not None:
        B = np.asmatrix(B)
    if C is not None:
        C = np.asmatrix(C)
    if D is not None:
        D = np.asmatrix(D)
    if Q_noise is not None:
        Q_noise = np.asmatrix(Q_noise)
    if R_noise is not None:
        R_noise = np.asmatrix(R_noise)
    if K is not None:
        K = np.asmatrix(K)
    if L is not None:
        L = np.asmatrix(L)
    if Kff is not None:
        Kff = np.asmatrix(Kff)
    if N is not None:
        N = np.asmatrix(N)

    if A is not None:
        assert A.shape[0] == A.shape[1],                                            \
            "A must be square"

    if A is not None and B is not None:
        assert B.shape[0] == A.shape[0],                                            \
            'A and B must have the same number of rows'

    if A is not None and C is not None:
        assert C.shape[1] == A.shape[0],                                            \
            'C must have as many columns as there are states'

    if D is not None and C is not None:
        assert D.shape[0] == C.shape[0],                                            \
            'C and D must have the same number of rows'
    if D is not None and B is not None:
        assert D.shape[1] == B.shape[1],                                            \
            'B and D must have the same number of columns'

    if Q_noise is not None and A is not None:
        assert Q_noise.shape == A.shape,                                            \
            'Q must have the same dimensions as A'

    if R_noise is not None:
        assert R_noise.shape[0] == R_noise.shape[1],                                \
            'R must be square'
    if R_noise is not None and C is not None:
        assert R_noise.shape[0] == C.shape[0],                                      \
            'R must have the same number of rows as there are sensor inputs'

    if K is not None and B is not None:
        assert K.shape[0] == B.shape[1],                                            \
            'K must have the same number of rows as there are inputs'
    if K is not None and A is not None:
        assert K.shape[1] == A.shape[0],                                            \
            'K must have the same number of columns as there are states'

    if L is not None and A is not None:
        assert L.shape[0] == A.shape[0],                                            \
            'L must have the same number of rows as there are states'
    if L is not None and C is not None:
        assert L.shape[1] == C.shape[0],                                            \
            'L must have the same number of columns as there are sensor inputs'

    if Kff is not None and A is not None:
        assert Kff.shape[1] == A.shape[1],                                          \
            'Kff must have the same number of columns as there are states'
    if Kff is not None and B is not None:
        assert Kff.shape[0] == B.shape[1],                                          \
            'Kff must have the same number of rows as there are inputs'

    if N is not None and C is not None:
        assert N.shape[1] == C.shape[0],                                            \
            'N must have the same number of columns as there are sensor inputs'
    if N is not None and B is not None:
        assert N.shape[0] == B.shape[1],                                            \
            'N must have the same number of rows as there are inputs'


def place_poles(A, B, poles):

    A = np.asmatrix(A)
    B = np.asmatrix(B)
    check_validity(A=A, B=B)
    if isinstance(poles, float):
        poles = [poles]
    else:
        assert isinstance(poles, list),                                             \
            'Poles must be either a single float or a list of floats or complex numbers'
        for pole in poles:
            if isinstance(pole, complex):
                assert pole.conjugate() in poles, 'Complex poles must form conjugate pairs'
            else:
                assert isinstance(pole, float),                                     \
                    'Poles must be complex conjugate pairs or floats'

    assert len(poles) == A.shape[0], 'The number of poles must equal the number of states'
    result = scipy.signal.place_poles(A, B, poles)
    print(result.gain_matrix)

    for requested, computed in zip(result.requested_poles, result.computed_poles):
        if abs(requested - computed) >= 1e-8:
            print('Requested pole %s could not be assigned and instead %s was assigned' % (requested, computed))

    return np.asmatrix(result.gain_matrix)


def controllability(A, B):
    """ Creates the controllability matrix from matrices A and C
        If the controllability matrix has full rank, then the system is completely controllable"""

    check_validity(A=A, B=B)

    n = B.shape[1]
    m = A.shape[1]
    ctrb = np.asmatrix(np.zeros((m, m*n)))
    current_submatrix = np.asmatrix(B)

    for i in range(m):
        ctrb[:m, i*n:i*n+n] = current_submatrix
        current_submatrix = A * current_submatrix

    return ctrb


def observability(A, C):
    """ Creates the observability matrix from matrices A and C
        If the observability matrix has full rank, then the system is completely observable"""

    check_validity(A=A, C=C)

    n = C.shape[0]
    m = A.shape[1]
    obsv = np.asmatrix(np.zeros((n*m, m)))
    current_submatrix = np.asmatrix(C)

    for i in range(m):
        obsv[i*n:i*n+n, :m] = current_submatrix
        current_submatrix = current_submatrix * A

    return obsv


def c2d(A, B, Q_noise, R_noise, dt: float):
    """ Convert a continuous-time dynamical system to a discrete time system
        Continuous-time form: dx(t)/t = A*x(t) + B*u(t), where x is a state vector and u is control input
        Discrete-time form: x[k+1] = A*x[k] + B*u[k], where k is an incrementing integer according to preset time steps
    """

    A = np.asmatrix(A)
    B = np.asmatrix(B)
    Q_noise = np.asmatrix(Q_noise)
    R_noise = np.asmatrix(R_noise)
    check_validity(A=A, B=B, Q_noise=Q_noise, R_noise=R_noise)

    n = A.shape[0]
    m = B.shape[1]

    M = np.zeros((n+m, n+m))
    M[:n, :n] = A
    M[:n, n:n+m] = B
    N = np.asmatrix(scipy.linalg.expm(M * dt))

    A_discrete = N[:n, :n]
    B_discrete = N[:n, n:n+m]

    F = np.zeros((n+n, n+n))
    F[:n, :n] = -A
    F[n:, n:] = A.T
    F[:n, n:n+n] = Q_noise
    G = np.asmatrix(scipy.linalg.expm(F * dt))

    Q_noise_discrete = A * G[:n, n:n+n]
    R_noise_discrete = R_noise / dt

    return [A_discrete, B_discrete, Q_noise_discrete, R_noise_discrete]


def clqr(A, B, Q_weight, R_weight):
    """ Return the optimal gain matrix K for controlling the continuous-time system
        according to weight matrices Q_weight and R_weight """

    A = np.asmatrix(A)
    B = np.asmatrix(B)
    Q_weight = np.asmatrix(Q_weight)
    R_weight = np.asmatrix(R_weight)
    check_validity(A=A, B=B)

    assert np.linalg.matrix_rank(controllability(A, B)) == A.shape[0],              \
        'System must be completely controllable to compute LQR gain matrix'

    # Use scipy's majik powers to solve the Ricatti equation
    P = np.asmatrix(scipy.linalg.solve_continuous_are(A, B, Q_weight, R_weight))

    # Use the matrix that you get from solving the Ricatti equation to solve for the optimal gain matrix K
    # K = R^-1 * B.T * P
    return np.asmatrix(scipy.linalg.inv(R_weight) * B.T * P)


def dlqr(A, B, Q_weight, R_weight):
    """ Return the optimal gain matrix K for controlling the discrete-time system
        according to weight matrices Q_weight and R_weight """

    A = np.asmatrix(A)
    B = np.asmatrix(B)
    Q_weight = np.asmatrix(Q_weight)
    R_weight = np.asmatrix(R_weight)
    check_validity(A=A, B=B)

    assert np.linalg.matrix_rank(controllability(A, B)) == A.shape[0],              \
        'System must be completely controllable to compute LQR gain matrix'

    # Use scipy's majik powers to solve the Ricatti equation
    P = np.asmatrix(scipy.linalg.solve_discrete_are(A, B, Q_weight, R_weight))

    # Use the matrix that you get from solving the Ricatti equation to solve for the optimal gain matrix K
    # K = (R + B.T * P * B)^-1 * B.T * P * A
    return np.asmatrix(scipy.linalg.inv(R_weight + B.T * P * B) * B.T * P * A)


def discrete_kalman(A, C, Q_noise, R_noise):
    """ Returns the optimal Kalman gain L according to the covariances and system and sensor dynamics
        This function is specifically for discrete-time systems"""

    A = np.asmatrix(A)
    C = np.asmatrix(C)
    Q_noise = np.asmatrix(Q_noise)
    R_noise = np.asmatrix(R_noise)
    check_validity(A=A, C=C, Q_noise=Q_noise, R_noise=R_noise)

    assert np.linalg.matrix_rank(observability(A, C)) == A.shape[0],                \
        'System must be completely observable to compute Kalman gains'

    # Applying lqr using A.T, C.T, Q, and R actually returns the transpose of the optimal Kalman gain L
    return np.asmatrix(dlqr(A.T, C.T, Q_noise, R_noise)).T


def continuous_kalman(A, C, Q_noise, R_noise):
    """ Returns the optimal Kalman gain L according to the covariances and system and sensor dynamics
        This function is specifically for continuous-time systems, and probably isn't actually very useful.
        But it was relatively easy to implement, so yay"""

    A = np.asmatrix(A)
    C = np.asmatrix(C)
    Q_noise = np.asmatrix(Q_noise)
    R_noise = np.asmatrix(R_noise)
    check_validity(A=A, C=C, Q_noise=Q_noise, R_noise=R_noise)

    assert np.linalg.matrix_rank(observability(A, C)) == A.shape[0],                \
        'System must be completely observable to compute Kalman gains'

    # Applying lqr using A.T, C.T, Q, and R actually returns the transpose of the optimal Kalman gain L
    return np.asmatrix(clqr(A.T, C.T, Q_noise, R_noise)).T


def tracking_gains_d(A_d, B_d, C, K_d):
    """
    Calculates the reference tracking gains in discrete time for tracking arbitrary step inputs
    In discrete time, N = - inv(C * inv(A - B*K - I) * B), where I is an n x n identity matrix, and n is the size of A
    """

    A_d = np.asmatrix(A_d)
    B_d = np.asmatrix(B_d)
    C = np.asmatrix(C)
    K_d = np.asmatrix(K_d)
    check_validity(A=A_d, B=B_d, C=C, K=K_d)

    n = A_d.shape[0]
    return np.asmatrix(-np.linalg.inv(-C * np.linalg.inv(A_d - B_d*K_d - np.identity(n)) * B_d))


def tracking_gains_c(A, B, C, K):
    """
    Calculates the reference tracking gains in continuous time for tracking arbitrary step inputs
    In continuous time, N = - inv(C * inv(A - B*K) * B)
    """

    A = np.asmatrix(A)
    B = np.asmatrix(B)
    C = np.asmatrix(C)
    K = np.asmatrix(K)
    check_validity(A=A, B=B, C=C, K=K)

    n = A_d.shape[0]
    return np.asmatrix(-np.linalg.inv(-C * np.linalg.inv(A - B*K) * B))


def feedforward_gains(B):
    """
    Calculate Kff for discrete-time according to x[k+1] = Ax[k] + B*uff, where uff = Kff * (x[k+1] - A*x[k])
    uff = pinv(B) * (x[k+1] - A*x[k]), so Kff = pinv(B)
    According to 1678 and 971, there's an LQR-weighted solution or something like that, but I'm averse to implementing
    things that I haven't seen a mathematical background for.
    """

    return np.linalg.pinv(B)

