from core.Status import StatusList
import numpy as np
import quaternion


class Actuator:
    def __init__(
        self,
        input_status_list: StatusList,
        output_status_list: StatusList,
        setting,
        timer,
    ):
        self.input_status_list = input_status_list
        self.output_status_list = output_status_list
        self.setting = setting
        self.obs_cnt = 0
        self.timer = timer

    def tick(self):
        self.update_state()

    def update_state(self):
        pass


class RWs(Actuator):
    def __init__(
        self,
        input_status_list: StatusList,
        output_status_list: StatusList,
        setting,
        timer,
    ):
        super(RWs, self).__init__(
            input_status_list, output_status_list, setting, timer
        )

    def update_state(self):
        torque_rw_dir = self.input_status_list.get_status("torque_rw_dir_Nm")
        rw_vel_rad_sec = self.output_status_list.get_status("rw_vel_rad_sec")
        torque_rw_t = np.array(
            [
                0
                if abs(rw_vel_rad_sec[i]) > self.setting.max_speed_rad_sec
                else (
                    torque_rw_dir[i]
                    if abs(torque_rw_dir[i]) < self.setting.max_torque_Nm
                    else np.sign(torque_rw_dir[i]) * self.setting.max_torque_Nm
                )
                for i in range(len(torque_rw_dir))
            ]
        )
        d_rw_vel_rad_sec = torque_rw_t / self.setting.inertia_kgm2
        rw_vel_rad_sec = rw_vel_rad_sec + d_rw_vel_rad_sec * self.timer.dt
        self.output_status_list.update_status("rw_vel_rad_sec", rw_vel_rad_sec)
        self.output_status_list.update_status("d_rw_vel_rad_sec", d_rw_vel_rad_sec)
        self.output_status_list.update_status("torque_rw_t_Nm", torque_rw_t)
