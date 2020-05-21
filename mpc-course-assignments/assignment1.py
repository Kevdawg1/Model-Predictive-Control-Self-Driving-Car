import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [12,12]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        a_t = pedal

        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + a_t * dt - v_t/25.0
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for k in range(0, self.horizon):
            v_state = state[3]
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            distance_diff = ref[0]-state[0]
            cost += abs(distance_diff)
            speed_kph = state[3]*3.6
            if (speed_kph > 10.0 and distance_diff < 15):
                cost += speed_kph*100
            if (speed_kph == 0 and distance_diff == 0):
                cost += 0
        return cost

sim_run(options, ModelPredictiveControl)
