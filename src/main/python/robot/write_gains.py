from utilities.state_space.gains_writer import GainsWriter

"""
This Python script is run before Java is compiled, so any constants or such that need to be generated via Python
should be generated here
"""

# Working directory when the gradle task is run defaults to project root
# Multiple output directories can be used in addition to multiple sets of gains
OUT_DIR = './src/main/java/frc/team687/robot/constants/'


def write_gains():
    pass
    # Create a GainsWriter from a GainsList
    # Write the gains to the files indicated by their names, in the directory indicated


if __name__ == '__main__':
    write_gains()

