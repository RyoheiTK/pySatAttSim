import numpy as np
import quaternion
import matplotlib.pyplot as plt
import copy
from SttSetting import SttSetting
from GyroSetting import GyroSetting
from scipy.integrate import odeint
from Status import Status, StatusList
from Sensor import Gyro, STT
from Actuator import RWs

class Satellite:
    def __init__(
        self,
        inertia,
        status_list,
        compo_list,
        logic_list
    ):
    # [TODO] 

    # Define Logics 


        self.inertia = inertia
        self.inertia_matrix = np.diag(self.inertia)
        self.status_list = status_list
        self.compo_list = compo_list
        self.logic_list = logic_list
        self.q = init_q
        self.omega = init_omega
        self.q_hist = [init_q]
        self.q_observe = init_q
        self.q_observe_hist = [init_q]
        self.q_observe_t_triad = init_q
        self.omega_observe_t = init_omega
        self.omegea_bias_estimate = 0
        self.omega_observe_hist = [init_omega]
        self.q_observe_triad = [np.array([1, 0, 0, 0])]
        self.q_observe_foam = [np.array([1, 0, 0, 0])]
        self.q_observe_kalm = init_q
        self.q_observe_kalm_hist = [init_q]

        self.gyro_setting = gyro_setting
        self.stt_setting = stt_setting
        self.omega_hist = [init_omega]
        self.omega_bias = gyro_setting.bias
        self.sat_sun_i = np.array([0, 0, 1])
        self.sat_mag_i = np.array([0, 0.6, 0.2])
        self.sat_sun_b_t = np.array([0, 0, 1])
        self.sat_mag_b_t = np.array([0, 1, 0])
        self.sat_sun_b_o = np.array([0, 0, 1])
        self.sat_mag_b_o = np.array([0, 1, 0])
        self.r2, self.r3 = self.calc_ref_vector()

        self.matrix_i_b = np.eye(3)
        self.matrix_i_b_estimated = np.eye(3)
        self.sun_o_bias = np.array([0, 0, 0])
        self.mag_o_bias = np.array([0, 0, 0])
        self.rw_vel = init_rw_vel
        self.rw_vel_hist = [init_rw_vel]
        self.rw_inertia = 0.5 * 1 / 6500 / 2 / np.pi * 60
        self.gain_setting = gain_setting
        self.q_target = quaternion.as_quat_array(np.array([1, 0, 0, 0]))
        self.u_hist = [np.array([0, 0, 0])]
        self.u = np.array([0, 0, 0])
        self.d_rw_vel = np.array([0, 0, 0])
        self.evep_hist = []
        self.timer = timer
        self.gyro_count = 0
        self.stt_count = 0
        self.est_att_update_period = est_att_update_period
        self.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]])
        self.Kt = np.zeros([3, 6])
        self.q_estimate = self.q_observe
        self.omega_estimate = self.omega_observe_t
        self.omega_bias_estimate = self.omega_bias
        self.aft_est_covariance_hist = []
        self.Kt_hist = []
        self.q_estimate_hist = [self.q_estimate]
        self.omega_bias_estimate_hist = [self.omega_bias_estimate]
        self.omega_estimate_hist = [self.omega_estimate]
        self.update_covariance_hist = []
        self.estimate_count = 0
        self.update_count = 0
        self.dynam_update_period = dynam_update_period
        self.B = np.array([[-0.5, 0, 0, 0, 0, 0],
                           [0, -0.5, 0, 0, 0, 0],
                           [0, 0, -0.5, 0, 0, 0],
                           [0, 0,    0, 1, 0, 0],
                           [0, 0,    0, 0, 1, 0],
                           [0, 0,    0, 0, 0, 1]])
        self.observe_covariance = self.calc_observe_covariance()
        self.disturb_covariance = np.zeros([6, 6])
        self.covariance = np.ones([6,6])*10

    def calc_observe_covariance(self):
        in_plane_x = self.stt_setting.in_plane_1sig
        in_plane_y = self.stt_setting.in_plane_1sig
        sight_direction_z = self.stt_setting.sight_direct_1sig
        q_observe = quaternion.from_rotation_vector(
            [in_plane_x, in_plane_y, sight_direction_z]
        )
        R_matrix = np.array([[q_observe.x, 0, 0],
                             [          0, q_observe.y, 0],
                             [          0,           0, q_observe.z]])
        return R_matrix

    def reserve_current_state(self):
        self.omega_hist.append(self.omega)
        self.q_hist.append(self.q)

    def set_vectors(self, sat_sun_i, mag_i):
        self.sat_sun_i = sat_sun_i
        self.sat_mag_i = mag_i

    def update_true_vec(self):
        q = quaternion.as_quat_array(self.q)
        self.matrix_i_b = quaternion.as_rotation_matrix(q)
        self.sat_sun_b_t = np.dot(self.matrix_i_b, self.sat_sun_i)
        self.sat_mag_b_t = np.dot(self.matrix_i_b, self.sat_mag_i)

    def update_observe_vec(self):
        self.sun_o_bias = 0.003 * np.random.randn(3) + self.sun_o_bias
        self.mag_o_bias = 0.02 * np.random.randn(3) + self.mag_o_bias
        self.sat_sun_b_o = self.sat_sun_b_t + self.sun_o_bias
        self.sat_mag_b_o = self.sat_mag_b_t + self.mag_o_bias

    def calc_dynamic_diff(self, x, t):
        w = x[0:3]
        q = x[3:7]
        w_dot = self.calc_w_dot(
            w, self.satellite.inertia_matrix, self.satellite.inertia
        )
        q_dot = self.calc_q_dot(q, w)
        dx = np.concatenate([w_dot, q_dot])
        return dx

    def update_state_vector(self):
        t = np.linspace(0, self.dynam_update_period, 10)
        self.omega_bias_estimate = odeint(self.calc_w_bias_dot, self.omega_bias_estimate, t)[-1]
        x = np.concatenate([self.omega_observe_t - self.omega_bias_estimate, self.q_estimate])
        x = odeint(self.calc_dynamic_diff, x, t)
        self.omega_estimate = x[-1][0:3]
        q = x[-1][3:7]
        self.q_estimate = (q) / np.linalg.norm(q)
        self.q_estimate_hist.append(self.q_estimate)
        self.omega_estimate_hist.append(self.omega_estimate)
        self.omega_bias_estimate_hist.append(self.omega_bias_estimate)

    def time_update_covariance(self):
        Ft = np.eye(6,6) + self.dynam_update_period * self.calc_A()
        Gt = self.dynam_update_period * self.B
        self.covariance = Ft.dot(self.covariance).dot(Ft.T) + Gt.dot(
            self.disturb_covariance
        ).dot(Gt.T)
        self.update_covariance_hist.append(self.covariance)

    def update_state(self):
        count = int(self.timer.get_time() / self.dynam_update_period)
        if count > self.update_count:
            self.update_state_vector()
            self.time_update_covariance()     
            self.update_count = count
        else:
            self.q_estimate_hist.append(self.q_estimate)
            self.omega_estimate_hist.append(self.omega_estimate)
            self.omega_bias_estimate_hist.append(self.omega_bias_estimate)
    def calc_A(self):
        w_bx = self.omega_bias_estimate[0]
        w_by = self.omega_bias_estimate[1]
        w_bz = self.omega_bias_estimate[2]
        tau = self.gyro_setting.omega_ecrv_tau
        A_matrix = np.array([[0,    w_bz, -w_by, -0.5,   0 ,    0], 
                      [-w_bz,   0,  w_bx,    0, -0.5,    0],
                      [w_by,-w_bx,     0,    0,    0, -0.5],
                      [   0,    0,     0,-1/tau,    0,    0],
                      [   0,    0,     0,     0,-1/tau,    0],
                      [   0,    0,     0,     0,    0,-1/tau]])
        return A_matrix

    def estimate_state(self):
        count = int(self.timer.get_time() / self.est_att_update_period)
        if count > self.estimate_count:   
            self.estimate_count =count
            q_estimate_q = quaternion.as_quat_array(self.q_estimate)     
            temp_obs =  q_estimate_q.inverse()* quaternion.as_quat_array(self.q_observe)
            z = np.array([temp_obs.x, temp_obs.y, temp_obs.z])
            self.Kt = self.covariance.dot(self.H.T).dot(np.linalg.inv(
                (
                    self.H.dot(self.covariance).dot(self.H.T) + self.observe_covariance
                ))
            )
            estimated_value = self.Kt.dot(z)
            quat = estimated_value[0:3]
            theta = np.sqrt(1-np.linalg.norm(quat))
            q_estimate_up = q_estimate_q * quaternion.as_quat_array(np.concatenate([np.array([theta]), quat]))
            self.q_estimate = quaternion.as_float_array(q_estimate_up)
            self.q_estimate = self.q_estimate/np.linalg.norm(self.q_estimate)
            self.omega_bias_estimate = self.omega_bias_estimate + estimated_value[3:6]
            self.covariance = np.dot(
                np.dot((np.eye(6,6) - self.Kt.dot(self.H)), self.covariance),
                (np.eye(6,6) - self.Kt.dot(self.H)).T,
            ).T + self.Kt.dot(self.observe_covariance).dot(self.Kt.T)
            self.aft_est_covariance_hist.append(self.covariance)
            self.Kt_hist.append(self.Kt)




    def calc_dynamic_diff(self, x, t):
        w = x[0:3]
        q = x[3:7]
        w_dot = self.calc_w_dot(
            w, self.inertia_matrix, self.inertia
        )
        q_dot = self.calc_q_dot(q, w)
        dx = np.concatenate([w_dot, q_dot])
        return dx

    def calc_w_bias_dot(self, w_bias, t):
        return -1 / self.gyro_setting.omega_ecrv_tau * w_bias

    def calc_w_dot(self, w, Inertia, It):
        w_dot = (
            -np.cross(
                w,
                (
                    np.dot(Inertia, w)
                    + self.rw_vel * self.rw_inertia
                ),
            )
            / It            + self.u / It
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

    def calc_ref_vector(self):
        r2, r3 = calc_perp_vec(self.sat_sun_i, self.sat_mag_i)
        return r2, r3

    def estimate_att_by_triad(self):
        b2, b3 = calc_perp_vec(self.sat_sun_b_o, self.sat_mag_b_o)
        r_matrix = np.array([self.sat_mag_i, self.r2, self.r3]).T
        b_matrix = np.array([self.sat_mag_b_o, b2, b3]).T
        R = np.dot(b_matrix, np.linalg.inv(r_matrix))
        q = quaternion.from_rotation_matrix(R, nonorthogonal=True)
        q_array = quaternion.as_float_array(q)
        self.q_observe_t_triad = q_array
        self.q_observe_triad.append(q_array)

    def observe_quat(self):
        count = int(self.timer.get_time() / self.stt_setting.update_period)
        if count > self.stt_count:
            self.stt_count = count
            in_plane_x = np.random.randn() * self.stt_setting.in_plane_1sig
            in_plane_y = np.random.randn() * self.stt_setting.in_plane_1sig
            sight_direction_z = np.random.randn() * self.stt_setting.sight_direct_1sig
            q_observe = quaternion.from_rotation_vector(
                [in_plane_x, in_plane_y, sight_direction_z]
            ) * quaternion.as_quat_array(self.q)
            self.q_observe = quaternion.as_float_array(q_observe)
            self.q_observe_hist.append(self.q_observe)
        else:
            self.q_observe_hist.append(self.q_observe)

    def observe_omega(self):
        count = int(self.timer.get_time() / self.gyro_setting.update_period)
        if count > self.gyro_count:
            self.gyro_count = count
            self.omega_bias += self.gyro_setting.random_walk * np.random.randn(
                len(self.omega)
            )
            self.omega_observe_t = (
                self.omega
                + self.omega_bias
                + self.gyro_setting.white_noise * np.random.randn(len(self.omega))
            )
            self.omega_observe_hist.append(self.omega_observe_t)
        else:
            self.omega_observe_hist.append(self.omega_observe_t)

    def _calc_torque_by_liapknov(self):  # 制御手法を選択 liapknov安定性に基づくやつ (佐々木さんの論文)
        vel_factor = np.dot(self.gain_setting.komega, self.omega)
        current_q = quaternion.as_quat_array(self.q)
        qe = current_q.inverse() * self.q_target
        q_array = quaternion.as_float_array(qe)
        q_factor = self.gain_setting.kq * q_array[1:4]
        u_candidate = -vel_factor - q_factor
        return u_candidate
"""
    def _calc_torque_three_axis(self):
        vel_factor = np.dot(self.gain_setting.komega, self.omega_estimate)
        try:
            omega_dot = (self.omega_estimate - self.omega_estimate_hist[-2]) / 0.0025
        except:
            omega_dot = (self.omega_estimate ) / 0.0025
        vel_factor_dot = np.dot(self.gain_setting.kdomega, omega_dot)
        current_q = quaternion.as_quat_array(self.q_estimate )
        qe = current_q.inverse() * self.q_target
        q_array = quaternion.as_float_array(qe)
        q_array = q_array/np.linalg.norm(q_array)
        if not(np.linalg.norm(q_array[1:4]) <0.0001):  
            evp = q_array[1:4] / np.linalg.norm(q_array[1:4])
        else:
            evp = np.array([0,0,1])
        ep = 2 * np.arccos(q_array[0])
        if ep > 2 * np.pi:
            ep = ep - 2 * np.pi
        elif ep < -2 * np.pi:
            ep = ep + 2 * np.pi
        if ep < 0:
            ep = -ep
        self.evep_hist.append(ep * evp)
        q_factor = ep * np.dot(self.gain_setting.kq, evp)
        integral = np.array([0, 0, 0])
        if len(self.evep_hist) > 4:
            integral = sum(self.evep_hist[-4:-1]) * 0.0025
        qi_factor = np.dot(self.gain_setting.kqi, integral)
        u_candidate = -vel_factor - vel_factor_dot - q_factor - qi_factor
        return u_candidate

    def _calc_gyro_torque(self):
        h_sat = np.dot(self.inertia_matrix, self.omega)
        h_rw = self.rw_inertia * self.rw_vel
        h_all = h_sat + h_rw
        gyro_torque = np.cross(self.omega, h_all)
        return gyro_torque

    def calc_torque(self):
        # u_candidate = self._calc_torque_by_liapknov()
        u_candidate = self._calc_torque_three_axis()
        gyro_torque = self._calc_gyro_torque()
        u_candidate += gyro_torque
        self.u = np.array(
            [
                u_candidate[i]
                if abs(u_candidate[i]) < 0.025
                else np.sign(u_candidate[i]) * 0.025
                for i in range(len(u_candidate))
            ]
        )
        # print(self.u)
        self.u_hist.append(self.u)
        self.d_rw_vel = self.u / self.rw_inertia
"""
    def update_rw(self, dt):
        self.rw_vel = self.rw_vel + self.d_rw_vel * dt
        self.rw_vel_hist.append(self.rw_vel)

    def estimate_att_by_foam(self):
        b1 = self.sat_mag_b_o
        b2, b3 = calc_perp_vec(self.sat_sun_b_o, self.sat_mag_b_o)
        a1 = 0.8
        a2 = 0.2
        r1 = self.sat_mag_i
        ramda = np.sqrt(
            a1 ** 2
            + a2 ** 2
            + 2
            * a1
            * a2
            * (
                np.dot(b1, b2) * np.dot(r1, self.r2)
                + np.linalg.norm(np.cross(b1, b2))
                * np.linalg.norm(np.cross(r1, self.r2))
            )
        )
        R = (
            np.dot(n(b3).T, n(self.r3))
            + (a1 / ramda)
            * (
                np.dot(n(b1).T, n(r1))
                + np.dot(n(np.cross(b1, b3)).T, n(np.cross(r1, self.r3)))
            )
            + (a2 / ramda)
            * (
                np.dot(n(b2).T, n(self.r2))
                + np.dot(n(np.cross(b2, b3)).T, n(np.cross(self.r2, self.r3)))
            )
        )
        q = quaternion.from_rotation_matrix(R, nonorthogonal=True)
        q_array = quaternion.as_float_array(q)
        self.q_observe_foam.append(q_array)


def n(vec):
    return np.array([vec])


def calc_perp_vec(vec0, vec1):
    r2 = np.cross(vec0, vec1)
    r2 = r2 / np.linalg.norm(r2)
    r3 = np.cross(vec0, r2)
    r3 = r3 / np.linalg.norm(r3)
    return r2, r3


class Sat_Visualizer:
    def __init__(self, dt, satellite):
        self.dt = dt
        self.satellite = satellite

    def quat_to_error_deg(self, q):
        current_q = quaternion.as_quat_array(q)
        qe = current_q.inverse() * self.satellite.q_target
        q_array = quaternion.as_float_array(qe)
        ep = 2 * np.arccos(q_array[0]) * 180 / np.pi
        if ep > 180:
            ep = 360 - ep
        return ep

    def visualize_omega(self):
        axs = []
        fig = plt.figure()
        labels = ["w0", "w1", "w2"]
        for i in range(3):
            axs.append(fig.add_subplot(3, 1, i + 1))
        t = np.arange(0, len(self.satellite.q_hist), 1) * self.dt
        for i in range(3):
            w_i = []
            w_obs = []
            w_est = []
            for j in range(len(self.satellite.omega_hist)):
                w_i.append(self.satellite.omega_hist[j][i] * 180 / np.pi)
                w_obs.append(self.satellite.omega_observe_hist[j][i] * 180 / np.pi)
                w_est.append(self.satellite.omega_estimate_hist[j][i] * 180 / np.pi)
            gatherd_t = copy.deepcopy(w_i)
            gatherd_obs = copy.deepcopy(w_obs)
            gatherd_est = copy.deepcopy(w_est)

            axs[i].set_xlabel("time[s]")  # x軸ラベル
            axs[i].set_ylabel("angular velocity [deg/sec]")  # y軸ラベル
            axs[i].grid()
            axs[i].plot(t, gatherd_obs, color="blue", label=labels[i] + "_o")
            axs[i].plot(t, gatherd_t, color="green", label=labels[i] + "_t")
            axs[i].plot(t, gatherd_est, color="red", label=labels[i] + "_e")
            axs[i].legend(loc=0)  # 凡例
        plt.show()

    def visualize_quat(self):
        axs = []
        fig = plt.figure()
        labels = ["w0", "w1", "w2", "w3"]
        for i in range(4):
            axs.append(fig.add_subplot(4, 1, i + 1))
        t = np.arange(0, len(self.satellite.q_observe_hist), 1) * self.dt
        for i in range(4):
            w_i = []
            w_obs = []
            w_est = []
            for j in range(len(self.satellite.q_observe_hist)):
                w_i.append(self.satellite.q_hist[j][i])
                w_obs.append(self.satellite.q_observe_hist[j][i])
                w_est.append(self.satellite.q_estimate_hist[j][i])
            gatherd_t = copy.deepcopy(w_i)
            gatherd_obs = copy.deepcopy(w_obs)
            gatherd_est = copy.deepcopy(w_est)

            axs[i].set_xlabel("time[s]")  # x軸ラベル
            axs[i].set_ylabel("Quaternion")  # y軸ラベル
            axs[i].grid()
            axs[i].plot(t, gatherd_obs, color="blue", label=labels[i] + "_o")
            axs[i].plot(t, gatherd_t, color="green", label=labels[i] + "_t")
            axs[i].plot(t, gatherd_est, color="red", label=labels[i] + "_e")
            axs[i].legend(loc=0)  # 凡例
        plt.show()

    def visualize_sat_motion(self, is_show=0):
        deg = []
        for j in range(len(self.satellite.q_hist)):
            q = self.satellite.q_hist[j]
            e_deg = self.quat_to_error_deg(q)
            deg.append(e_deg)
        if is_show:
            fig = plt.figure()
            ax1 = fig.add_subplot(3, 1, 1)
            t = np.arange(0, len(self.satellite.q_hist), 1) * self.dt
            c1, c2, c3, c4 = "blue", "green", "red", "black"  # 各プロットの色
            l1, l2, l3, l4 = "q0", "q1", "q2", "q3"  # 各ラベル
            q = []
            for i in range(4):
                q_i = []
                for j in range(len(self.satellite.q_hist)):
                    q_i.append(self.satellite.q_hist[j][i])
                gathered = copy.deepcopy(q_i)
                q.append(gathered)

            ax1.set_xlabel("time[s]")  # x軸ラベル
            ax1.set_ylabel("quaternion")  # y軸ラベル
            ax1.grid()
            ax1.plot(t, q[0], color=c1, label=l1)
            ax1.plot(t, q[1], color=c2, label=l2)
            ax1.plot(t, q[2], color=c3, label=l3)
            ax1.plot(t, q[3], color=c4, label=l4)
            ax1.legend(loc=0)  # 凡例

            ax2 = fig.add_subplot(3, 1, 2)
            w = []
            for i in range(3):
                w_i = []
                for j in range(len(self.satellite.omega_hist)):
                    w_i.append(self.satellite.omega_hist[j][i] * 180 / np.pi)
                gathered = copy.deepcopy(w_i)
                w.append(gathered)

            l1, l2, l3 = "w0", "w1", "w2"  # 各ラベル
            ax2.set_xlabel("time[s]")  # x軸ラベル
            ax2.set_ylabel("angular velocity [deg/sec]")  # y軸ラベル
            ax2.grid()
            ax2.plot(t, w[0], color=c1, label=l1)
            ax2.plot(t, w[1], color=c2, label=l2)
            ax2.plot(t, w[2], color=c3, label=l3)
            ax2.legend(loc=0)  # 凡例

            ax1 = fig.add_subplot(3, 1, 3)
            t = np.arange(0, len(self.satellite.q_hist), 1) * self.dt
            c1, c2, c3, c4 = "blue", "green", "red", "black"  # 各プロットの色
            l1, l2, l3, l4 = "q0", "q1", "q2", "q3"  # 各ラベル

            ax1.set_xlabel("time[s]")  # x軸ラベル
            ax1.set_ylabel("Error [deg]")  # y軸ラベル
            ax1.grid()
            ax1.plot(t, deg)
            ax1.legend(loc=0)  # 凡例

            fig.tight_layout()  # レイアウトの設定
            # plt.savefig('hoge.png') # 画像の保存
            plt.show()
        return deg

    def compare_observation(self):
        fig = plt.figure()
        ax1 = fig.add_subplot(3, 1, 1)
        t = np.arange(0, len(self.satellite.q_hist), 1) * self.dt
        c1, c2, c3, c4 = "blue", "green", "red", "black"  # 各プロットの色
        l1, l2, l3, l4 = "q0", "q1", "q2", "q3"  # 各ラベル
        q = []
        for i in range(4):
            q_i = []
            for j in range(len(self.satellite.q_hist)):
                q_i.append(self.satellite.q_hist[j][i])
            gathered = copy.deepcopy(q_i)
            q.append(gathered)

        ax1.set_xlabel("time[s]")  # x軸ラベル
        ax1.set_ylabel("quaternion")  # y軸ラベル
        ax1.grid()
        ax1.plot(t, q[0], color=c1, label=l1)
        ax1.plot(t, q[1], color=c2, label=l2)
        ax1.plot(t, q[2], color=c3, label=l3)
        ax1.plot(t, q[3], color=c4, label=l4)
        ax1.legend(loc=0)  # 凡例

        ax2 = fig.add_subplot(3, 1, 2)
        t = np.arange(0, len(self.satellite.q_observe_triad), 1) * self.dt
        c1, c2, c3, c4 = "blue", "green", "red", "black"  # 各プロットの色
        l1, l2, l3, l4 = "q0", "q1", "q2", "q3"  # 各ラベル
        q = []
        for i in range(4):
            q_i = []
            for j in range(len(self.satellite.q_observe_triad)):
                q_i.append(self.satellite.q_observe_triad[j][i])
            gathered = copy.deepcopy(q_i)
            q.append(gathered)

        ax2.set_xlabel("time[s]")  # x軸ラベル
        ax2.set_ylabel("quaternion (Observation)")  # y軸ラベル
        ax2.grid()
        ax2.plot(t, q[0], color=c1, label=l1)
        ax2.plot(t, q[1], color=c2, label=l2)
        ax2.plot(t, q[2], color=c3, label=l3)
        ax2.plot(t, q[3], color=c4, label=l4)
        ax2.legend(loc=0)  # 凡例

        ax3 = fig.add_subplot(3, 1, 3)
        t = np.arange(0, len(self.satellite.q_observe_triad), 1) * self.dt
        c1, c2, c3, c4 = "blue", "green", "red", "black"  # 各プロットの色
        l1, l2, l3, l4 = "q0", "q1", "q2", "q3"  # 各ラベル
        q = []
        for i in range(4):
            q_i = []
            for j in range(len(self.satellite.q_observe_foam)):
                q_i.append(self.satellite.q_observe_foam[j][i])
            gathered = copy.deepcopy(q_i)
            q.append(gathered)

        ax3.set_xlabel("time[s]")  # x軸ラベル
        ax3.set_ylabel("quaternion (Observation foam)")  # y軸ラベル
        ax3.grid()
        ax3.plot(t, q[0], color=c1, label=l1)
        ax3.plot(t, q[1], color=c2, label=l2)
        ax3.plot(t, q[2], color=c3, label=l3)
        ax3.plot(t, q[3], color=c4, label=l4)
        ax3.legend(loc=0)  # 凡例

        fig.tight_layout()  # レイアウトの設定
        # plt.savefig('hoge.png') # 画像の保存
        plt.show()
        fig = plt.figure()
        ax1 = fig.add_subplot(1, 1, 1)
        t = np.arange(0, len(self.satellite.rw_vel_hist), 1) * self.dt
        c1, c2, c3 = "blue", "green", "red"
        l1, l2, l3 = "rwx", "rwy", "rwz"
        rw = []
        for i in range(3):
            rw_i = []
            for j in range(len(self.satellite.rw_vel_hist)):
                rw_i.append(self.satellite.rw_vel_hist[j][i] * 60 / 2 / np.pi)
            gathered = copy.deepcopy(rw_i)
            rw.append(gathered)

        ax1.set_xlabel("time[s]")  # x軸ラベル
        ax1.set_ylabel("RW Vel")  # y軸ラベル
        ax1.grid()
        ax1.plot(t, rw[0], color=c1, label=l1)
        ax1.plot(t, rw[1], color=c2, label=l2)
        ax1.plot(t, rw[2], color=c3, label=l3)
        ax1.legend(loc=0)  # 凡例
        plt.show()
