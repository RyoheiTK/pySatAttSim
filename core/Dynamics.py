import numpy as np
from scipy.integrate import odeint

class Dynamics:
    def __init__(self, dt, satellite, rw_inertia):
        self.satellite = satellite
        self.rw_inertia = rw_inertia
        self.dt = dt
        self.t = 0

    def update_time(self):
        w = self.satellite.status_list.get_status("w_t_rad_sec")
        q = self.satellite.status_list.get_status("q_t")
        t = np.linspace(0,self.dt, 10)
        x = np.concatenate([w, q])
        x = odeint(self.calc_dynamic_diff,x, t)
        w_updated = x[-1][0:3]
        q = x[-1][3:7]
        q_updated = (q )/ np.linalg.norm(q )
        self.satellite.status_list.update_status("w_t_rad_sec", w_updated)
        self.satellite.status_list.update_status("q_t", q_updated)
        # w_dot = self.calc_w_dot(w, Inertia, It)
        # q_dot = self.calc_q_dot(q, w)
        # self.satellite.omega = w_dot * self.dt + w
        # self.satellite.q = (q + q_dot * self.dt) / np.linalg.norm(q + q_dot * self.dt)
        self.t += self.dt

    def calc_dynamic_diff(self, x, t):
        w = x[0:3]
        q = x[3:7]
        w_dot = self.calc_w_dot(w, self.satellite.inertia_matrix, self.satellite.inertia)
        q_dot = self.calc_q_dot(q, w)
        dx = np.concatenate([w_dot, q_dot])
        return dx

    def calc_w_dot(self, w, Inertia, It):
        rw_vel = self.satellite.status_list.get_status("rw_vel_rad_sec")
        torque_rw_t_Nm = self.satellite.status_list.get_status("torque_rw_t_Nm")
        gyro_torque = np.cross(w, (np.dot(Inertia, w) + rw_vel * self.rw_inertia))
        w_dot = -gyro_torque / It + torque_rw_t_Nm / It
        if np.isinf(w_dot[0]):
            print("here")
        return w_dot

    def calc_q_dot(self, q, w):
        q_dot = (
            1
            / 2
            * np.dot(
                np.array(
                    [
                        [0, -w[0], -w[1], -w[2]],
                        [w[0], 0, -w[2], -w[1]],
                        [w[1], w[2], 0, -w[0]],
                        [w[2], -w[1], w[0], 0],
                    ]
                ),
                q,
            )
        )
        return q_dot
