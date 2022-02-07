import numpy as np


class EKFGyroSTTSetting:
    def __init__(
        self,
        in_plane_1sig,
        sight_direct_1sig,
        update_period,
        est_period,
        omega_ecrv_tau,
        inertia,
        rw_inertia,
        dt, 
        num_step
    ):
        self.in_plane_1sig = in_plane_1sig
        self.sight_direct_1sig = sight_direct_1sig
        self.update_period = update_period
        self.est_period = est_period
        self.omega_ecrv_tau = omega_ecrv_tau
        self.rw_inertia = rw_inertia
        self.inertia = inertia
        self.inertia_matrix = np.diag(self.inertia)
        self.t_series = np.linspace(0, dt, num_step)
        
