

class GyroSetting:
    def __init__(self, random_walk, bias,  white_noise, omega_ecrv_tau, update_period):
        self.random_walk = random_walk
        self.bias = bias
        self.white_noise = white_noise
        self.omega_ecrv_tau = omega_ecrv_tau
        self.update_period = update_period