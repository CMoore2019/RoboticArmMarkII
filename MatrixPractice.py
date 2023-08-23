import numpy as np
from IK import IK
from workspacetest import workspacetest
from FK import FK
from MoveTo import MoveTo
from FindBasisVectors import FindBasisVectors
from catapult import catapult
import time

MoveTo(np.array([0,0,0,0]),0)
time.sleep(3)
land = np.array([11,-5])
start = np.array([11,0])
catapult(start,land)
