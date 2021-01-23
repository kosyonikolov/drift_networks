class Engine:
    def __init__(self, max_rpm, max_force):
        self.max_rpm = max_rpm
        self.max_force = max_force

    def get_force(self, rpm, throttle):
        return throttle * self.max_force
