import math
from enum import Enum

# RPM to rad/s, final unit is rad/s
RPM_TO_RADS_PER_S = 2. * math.pi / 60.


# Current in Amps
# Stall Torque in N-m
# Stall current in Amps
# Battery voltage in Volts
class MotorType(Enum):
    _PRO775 = [18730. * RPM_TO_RADS_PER_S, .7, .71, 134., 12.]
    _CIM = [5330. * RPM_TO_RADS_PER_S, 2.7, 2.41, 131., 12.]
    _MINI_CIM = [5840. * RPM_TO_RADS_PER_S, 3., 1.4, 89., 12.]
    _BAG = [13180. * RPM_TO_RADS_PER_S, 1.8, .4, 53., 12.]