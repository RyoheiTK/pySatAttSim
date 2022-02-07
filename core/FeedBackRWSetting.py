

class FeedBackRWSetting:
    def __init__(self, kq, kqi, komega, kdomega, inertia_matrix, rw_inertia, q_target):
        self.kq = kq
        self.kqi = kqi
        self.komega = komega
        self.kdomega = kdomega
        self.inertia_matrix = inertia_matrix
        self.rw_inertia = rw_inertia
        self.update_period = 0.05
        self.q_target = q_target
