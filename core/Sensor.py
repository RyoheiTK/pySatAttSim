from core.Status import StatusList
import numpy as np
import quaternion

class Senser:
    def __init__(self, input_status_list:StatusList, output_status_list:StatusList, setting, timer):
        self.input_status_list = input_status_list
        self.output_status_list = output_status_list
        self.setting = setting
        self.obs_cnt = 0
        self.timer = timer
    
    def tick(self):
        count = int(self.timer.get_time() / self.setting.update_period)
        if count > self.obs_cnt:
            self.obs_cnt = count
            self.observe()
        else:
            self.store_keep_val()

    def store_keep_val(self):
        [status.store_keep_val() for status in self.output_status_list.list]

    def observe(self):
        # 基本的に
        pass

class STT(Senser):
    def __init__(self, input_status_list:StatusList, output_status_list:StatusList, setting, timer):
        super(STT, self).__init__(input_status_list, output_status_list, setting, timer)

    def observe(self):
        in_plane_x = np.random.randn() * self.setting.in_plane_1sig
        in_plane_y = np.random.randn() * self.setting.in_plane_1sig
        sight_direction_z = np.random.randn() * self.setting.sight_direct_1sig
        q_t = self.input_status_list.get_status("q_t")
        q_observe = quaternion.from_rotation_vector(
            [in_plane_x, in_plane_y, sight_direction_z]
        ) * quaternion.as_quat_array(q_t)
        q_observe = quaternion.as_float_array(q_observe)
        self.output_status_list.update_status("q_stt_obs", q_observe)

class Gyro(Senser):
    def __init__(self, input_status_list:StatusList, output_status_list:StatusList, setting, timer):
        super(Gyro, self).__init__(input_status_list, output_status_list, setting, timer)
    def observe(self):
        w_t = self.input_status_list.get_status("w_t_rad_sec")
        w_bias_t = self.output_status_list.get_status("w_bias_t_rad_sec")
        w_bias = self.setting.random_walk * np.random.randn(
            len(w_t)
        ) + w_bias_t
        self.output_status_list.update_status("w_bias_t_rad_sec", w_bias)
        w_gyro_obs_val = (
            w_t
            + w_bias_t
            + self.setting.white_noise * np.random.randn(len(w_t))
        )
        self.output_status_list.update_status("w_gyro_obs_rad_sec", w_gyro_obs_val)