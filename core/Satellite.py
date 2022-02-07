import numpy as np
import quaternion
import matplotlib.pyplot as plt


class Satellite:
    def __init__(
        self,
        inertia,
        status_list,
        compo_list,
        logic_list
    ):
        self.inertia = inertia
        self.inertia_matrix = np.diag(self.inertia)
        self.status_list = status_list
        self.compo_list = compo_list
        self.logic_list = logic_list
        
    def tick(self):
        [compo.tick() for compo in self.compo_list]
        [logic.tick() for logic in self.logic_list]

 