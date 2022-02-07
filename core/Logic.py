#from statistics import covariance
from core.Status import StatusList
import numpy as np
import quaternion
from scipy.integrate import odeint
from copy import deepcopy

class Logic:
    def __init__(self, input_status_list: StatusList, output_status_list: StatusList, setting, timer):
        self.input_status_list = input_status_list
        self.output_status_list = output_status_list
        self.setting = setting
        self.obs_cnt = 0
        self.timer = timer

    def tick(self):
        count = int(self.timer.get_time() / self.setting.update_period)
        if count > self.obs_cnt:
            self.obs_cnt = count
            self.calc()
        else:
            self.store_keep_val()

    def store_keep_val(self):
        [status.store_keep_val() for status in self.output_status_list.list]

    def calc(self):
        # 基本的に
        pass


class FeedBackRW(Logic):
    def __init__(self, input_status_list: StatusList, output_status_list: StatusList, setting, timer):
        super(FeedBackRW, self).__init__(input_status_list,
                                    output_status_list, setting, timer)

    def calc(self):
        u_candidate = self._calc_torque_three_axis()
        gyro_torque = self._calc_gyro_torque()
        u_candidate += gyro_torque
        self.output_status_list.update_status("torque_rw_dir_Nm", u_candidate)

    def _calc_torque_three_axis(self):
        w_est_rad_sec = self.input_status_list.extract_status("w_est_rad_sec")
        q_est = self.input_status_list.get_status("q_est")
        evep = self.output_status_list.extract_status("evep")
        vel_factor = -np.dot(self.setting.komega, w_est_rad_sec.value)
        if len(w_est_rad_sec.history) > 4:
            omega_dot = (w_est_rad_sec.value - w_est_rad_sec.history[-2]) / self.setting.update_period
        else:
            omega_dot = (w_est_rad_sec.value) / self.setting.update_period
        vel_factor_dot = -np.dot(self.setting.kdomega, omega_dot)
        q_val = deepcopy(q_est)
        if q_val[0]<0:
            q_val = -deepcopy(q_est)
        current_q = quaternion.as_quat_array(q_val)

        qe = current_q.inverse() * quaternion.as_quat_array(self.setting.q_target)
        q_array = quaternion.as_float_array(qe)
        q_array = q_array/np.linalg.norm(q_array)
        if not(np.linalg.norm(q_array[1:4]) < 0.0001):
            evp = q_array[1:4] / np.linalg.norm(q_array[1:4])
        else:
            evp = np.array([0, 0, 1])
        ep = 2 * np.arccos(q_array[0])
        if ep > 2 * np.pi:
            ep = ep - 2 * np.pi
        elif ep < -2 * np.pi:
            ep = ep + 2 * np.pi
        if ep > np.pi:
            ep = -ep
        self.output_status_list.update_status("evep", ep*evp)
        q_factor = np.dot(self.setting.kq, ep*evp)
        integral = np.array([0, 0, 0])
        if len(evep.history) > 4:
            integral = sum(evep.history[-4:-1]) * self.setting.update_period
        qi_factor = -np.dot(self.setting.kqi, integral)
        u_candidate = vel_factor + vel_factor_dot + q_factor + qi_factor
        return u_candidate

    def _calc_gyro_torque(self):
        w_est_rad_sec = self.input_status_list.get_status("w_est_rad_sec")
        rw_vel_rad_sec = self.input_status_list.get_status("rw_vel_rad_sec")
        h_sat = np.dot(self.setting.inertia_matrix, w_est_rad_sec)
        h_rw = self.setting.rw_inertia * rw_vel_rad_sec
        h_all = h_sat + h_rw
        gyro_torque = np.cross(w_est_rad_sec, h_all)
        return gyro_torque


class ExtendedKalmanFilter(Logic):
    def __init__(self, input_status_list: StatusList, output_status_list: StatusList, setting, timer):
        super(ExtendedKalmanFilter, self).__init__(input_status_list,
                                    output_status_list, setting, timer)
        self.state = self.def_state()
        self.est_count = 0
        self.observe_covariance = None
        self.disturb_covariance = None
        self.H = None

    def def_state(self):
        pass

    def update_state_vector(self, state):
        updated_state = odeint(self.calc_dynamic_diff,
                            state, self.setting.t_series)[-1]
        return updated_state
    def calc_dynamic_diff(self, state, t):
        pass

    def calc_A(self, state):
        pass

    def calc_B(self, state):
        pass

    def time_update_state_covariance(self, state):
        state_covariance = self.output_status_list.get_status(
            "state_covariance")
        Ft = np.eye(6, 6) + self.setting.update_period * self.calc_A(state)
        Gt = self.setting.update_period * self.calc_B(state)
        state_covariance = Ft.dot(state_covariance).dot(Ft.T) + Gt.dot(
            self.disturb_covariance
        ).dot(Gt.T)
        return state_covariance

    def calc_Kt(self, state_covariance):
        Kt = state_covariance.dot(self.H.T).dot(np.linalg.inv(
            (
                self.H.dot(state_covariance).dot(
                    self.H.T) + self.observe_covariance
            ))
        )
        return Kt

    def calc_obs_result_covariance(self, Kt, state_covariance):
        state_covariance = np.dot(
            np.dot((np.eye(6, 6) - Kt.dot(self.H)), state_covariance),
            (np.eye(6, 6) - Kt.dot(self.H)).T,
        ).T + Kt.dot(self.observe_covariance).dot(Kt.T)
        return state_covariance

    def estimate(self, Kt, z):
        estimated_value = Kt.dot(z)
        return estimated_value

    def calc_z(self, state):
        pass

    def calc(self):
        state = self.def_state()
        state = self.update_state_vector(state)
        state_covariance = self.time_update_state_covariance(state)
        count = int(self.timer.get_time() / self.setting.est_period)
        if count > self.est_count:
            self.est_count = count
            Kt = self.calc_Kt(state_covariance)
            z = self.calc_z(state)
            est_state = self.estimate(Kt, z)
            updated_state = self.update_state(est_state, state)
            state_covariance = self.calc_obs_result_covariance(
                Kt, state_covariance)
            self.est_update_state(updated_state, state_covariance)
        else:
            self.keep_prop_state(state)

    def est_update_state(self, est_state, state_covariance):
        pass

    def keep_prop_state(self, state):
        pass

    def update_state(self, est_state, state):
        pass




class EKFGyroSTT(ExtendedKalmanFilter):
    def __init__(self, input_status_list: StatusList, output_status_list: StatusList, setting, timer):
        super(EKFGyroSTT, self).__init__(input_status_list,
                                    output_status_list, setting, timer)
        self.state = self.def_state()
        self.est_count = 0
        self.H = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]])
        self.observe_covariance = self.calc_observe_covariance()
        self.disturb_covariance = np.zeros([6, 6])

    def calc_observe_covariance(self):
        in_plane_x = self.setting.in_plane_1sig
        in_plane_y = self.setting.in_plane_1sig
        sight_direction_z = self.setting.sight_direct_1sig
        q_observe = quaternion.from_rotation_vector(
            [in_plane_x, in_plane_y, sight_direction_z]
        )
        R_matrix = np.array([[q_observe.x, 0, 0],
                             [0, q_observe.y, 0],
                             [0,           0, q_observe.z]])
        return R_matrix

    def def_state(self):
        q_est = self.output_status_list.get_status("q_est")
        q_est = q_est/np.linalg.norm(q_est) 
        w_est = self.output_status_list.get_status("w_est_rad_sec")
        w_bias_est = self.output_status_list.get_status("w_bias_est_rad_sec")
        state = np.concatenate([q_est, w_est, w_bias_est])
        return state

    def calc_dynamic_diff(self, state, t):
        rw_vel_rad_sec = self.input_status_list.get_status("rw_vel_rad_sec")
        torque_rw_t_Nm = self.input_status_list.get_status("torque_rw_t_Nm")
        q = state[0:4]
        w = state[4:7]
        w_bias = state[7:10]
        w_bias_dot = -1 / self.setting.omega_ecrv_tau * w_bias
        w_dot = self.calc_w_dot(
            w, self.setting.inertia_matrix, self.setting.inertia, rw_vel_rad_sec, torque_rw_t_Nm
        )
        q_dot = self.calc_q_dot(q, w)
        dx = np.concatenate([q_dot, w_dot, w_bias_dot])
        return dx

    def update_state(self, est_state, state):
        quat = est_state[0:3]
        theta = np.sqrt(1-np.linalg.norm(quat))
        q = state[0:4]
        q_estimate_up = quaternion.as_quat_array(q) * quaternion.as_quat_array(np.concatenate([np.array([theta]), quat]))
        q_update = quaternion.as_float_array(q_estimate_up)
        w_bias_estimate = state[7:10] + est_state[3:6]
        w_obs = self.input_status_list.get_status("w_gyro_obs_rad_sec")
        w_est = w_obs - w_bias_estimate
        state = np.concatenate([q_update, w_est, w_bias_estimate])
        return state       


    def calc_A(self, state):
        w_bias = state[7:10]
        w_bx = w_bias[0]
        w_by = w_bias[1]
        w_bz = w_bias[2]
        tau = self.setting.omega_ecrv_tau
        A_matrix = np.array([[0,    w_bz, -w_by, -0.5,   0,    0],
                             [-w_bz,   0,  w_bx,    0, -0.5,    0],
                             [w_by, -w_bx,     0,    0,    0, -0.5],
                             [0,    0,     0, -1/tau,    0,    0],
                             [0,    0,     0,     0, -1/tau,    0],
                             [0,    0,     0,     0,    0, -1/tau]])
        return A_matrix

    def calc_B(self, state):
        B_matrix = np.array([[-0.5, 0, 0, 0, 0, 0],
                             [0, -0.5, 0, 0, 0, 0],
                             [0, 0, -0.5, 0, 0, 0],
                             [0, 0,    0, 1, 0, 0],
                             [0, 0,    0, 0, 1, 0],
                             [0, 0,    0, 0, 0, 1]])
        return B_matrix

    def calc_z(self, state):
        q_observe = self.input_status_list.get_status("q_stt_obs")
        q_estimate = state[0:4]
        q_estimate_q = quaternion.as_quat_array(q_estimate)
        temp_obs = q_estimate_q.inverse() * quaternion.as_quat_array(q_observe)
        z = np.array([temp_obs.x, temp_obs.y, temp_obs.z])
        return z

    def est_update_state(self, est_state, state_covariance):
        self.keep_prop_state(est_state)
        self.output_status_list.update_status(
            "state_covariance", state_covariance)

    def keep_prop_state(self, state):
        q = state[0:4]
        w = state[4:7]
        w_bias = state[7:10]
        self.output_status_list.update_status("q_est", q)
        w_obs = self.input_status_list.get_status("w_gyro_obs_rad_sec")
        w_est = w_obs - w_bias
        self.output_status_list.update_status("w_est_rad_sec", w_est)
        self.output_status_list.update_status("w_bias_est_rad_sec", w_bias)
        
        
    def calc_w_dot(self, w, Inertia, It, rw_vel_rad_sec, u):
        w_dot = (
            -np.cross(
                w,
                (
                    np.dot(Inertia, w)
                    + rw_vel_rad_sec * self.setting.rw_inertia
                ),
            )
            / It            + u / It
        )
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
