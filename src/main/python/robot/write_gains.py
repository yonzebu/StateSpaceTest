from utilities.state_space.gains_writer import GainsWriter
from robot import motor_test
from robot import flywheel_test

"""
This Python script is run before Java is compiled, so any constants or such that need to be generated via Python
should be generated here
"""

# Working directory when the gradle task is run defaults to project root
# Multiple output directories can be used in addition to multiple sets of gains
OUT_DIR = './src/main/java/frc/team687/robot/constants/'


def write_gains():
    # Create a GainsWriter from a GainsList
    # In this instance, the subsystem in question is given its own individual Python file from which gains are created
    gains_list = motor_test.create_gains()[0]
    gains_list.add_gains(flywheel_test.create_gains()[0].get_gains(0))
    writer = GainsWriter(gains_list)
    # # Write the gains to the files indicated by their names, in the directory indicated
    writer.write_all([OUT_DIR, OUT_DIR])


if __name__ == '__main__':
    write_gains()

