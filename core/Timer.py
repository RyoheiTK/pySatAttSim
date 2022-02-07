
class Timer:
    def __init__(self, dt):
        self.time = 0
        self.dt = dt
    def update_time(self):
        self.time += self.dt

    def get_time(self):
        return self.time