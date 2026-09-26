class ZeroTorqueController:
    def update(self, simulator):
        simulator.data.ctrl[:]=0.0