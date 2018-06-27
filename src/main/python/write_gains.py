import numpy as np
from utilities.gains_writer import GainsWriter, numpy_to_jama_matrix
from utilities.state_space_gains import GainsList, StateSpaceGains, default_gains
from utilities.state_space_utils import observability, controllability
from robot import motor

# Working directory when the gradle task is run defaults to project root
# Multiple output directories can be used in addition to multiple sets of gains
OUT_DIR = '.\\src\\main\\java\\frc\\team687\\robot\\constants\\'

# Create a GainsWriter from a GainsList
# In this instance, the subsystem in question is given its own individual Python file from which gains are created
writer = GainsWriter(motor.create_gains())
# # Write the gains to the files indicated by their names, in the directory indicated
writer.write_all(OUT_DIR)
