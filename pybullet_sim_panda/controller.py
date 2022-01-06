import numpy as np
import pybullet as p
from pybullet_sim_panda.bullet_robot import PandaBullet

class TaskSpaceImpedanceControl:
    def __init__(self, robot, config):
        """ 
        init TaskSpaceImpedanceControl.
        robot : PandaBullet class
        config : a dictionary which contains "trans_k, rot_k"
        """
        self._trans_k = config["trans_k"]
        self._rot_k = config["rot_k"]
    
    def set_target(self):
        pass

    def _loop(self):
        pass