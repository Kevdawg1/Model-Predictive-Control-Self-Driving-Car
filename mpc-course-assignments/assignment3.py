import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = [0, 0, 0]

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        a_t = pedal
        beta = steering

        x_t_1 = x_t + (v_t * np.cos(psi_t)) * dt
        y_t_1 = y_t + (v_t * np.sin(psi_t)) * dt
        v_t_1 = v_t + a_t * dt - v_t/25
        psi_t_1 = psi_t + v_t * np.tan(beta) * dt

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for k in range(0, self.horizon):
            v_init = state[3]
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])

            distance_x_diff = ref[0]-state[0]
            cost += abs(distance_x_diff)**2

            distance_y_diff = ref[1]-state[1]
            cost += abs(distance_y_diff)**2

            angle_diff = ref[2] - state[2]
            cost += abs(angle_diff)**2

            # Obstacle Cost
            cost += self.obstacle_cost(state[0], state[1])
            # obs_dist_x = self.x_obs - state[0]
            # obs_dist_y = self.y_obs - state[1]
            # obs_dist_tot = np.sqrt([obs_dist_x**2+obs_dist_y**2])
            # cost += 1/(obs_dist_tot) * 8
            # # Acceleration Cost
            # v_curr = state[3]
            # cost += (v_curr - v_init)**2 * 50

            # Steering Input Cost
            cost += u[k*2+1]**2 * self.dt
        return cost

    def obstacle_cost(self, x, y):
        distance = (x - self.x_obs)**2 + (y - self.y_obs)**2
        distance = np.sqrt(distance)

        if (distance > 2):
            return 15
        else: 
            return 1/distance * 30

sim_run(options, ModelPredictiveControl)
