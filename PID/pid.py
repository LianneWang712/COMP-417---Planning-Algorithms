class PIDController:
    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 0.0 #7000.0
        self.Ki = 0.0 #90.0
        self.Kd = 0.0 #3400.0
        self.bias = 0.0

        self.integral = 0
        self.last_position = 0
        self.prev_output = 0

        return

    def reset(self):
        return

    # TODO: Complete your PID control within this function. At the moment, it holds
    #      only the bias. Your final solution must use the error between the
    #      target_pos and the ball position, plus the PID gains. You cannot
    #      use the bias in your final answer.
    def get_fan_rpm(self, vertical_ball_position):
        # use target position, vertical ball position, kp,ki,d.
        # u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        #error = vertical_ball_position - self.target_pos

        #calculate the error with e = current_state - target_state
        error = self.target_pos - vertical_ball_position
        error_position = vertical_ball_position - self.last_position

        # faster response but with overshoot and undershoot
        proportional = self.Kp * error

        # finite differences = e(tk) - e(tk-1)/delta
        # reduces both responsiveness and oscillations
        derivative = -self.Kd * error_position / error

        # handle biases/reduces steady state errors
        self.integral += self.Ki * error

        output = proportional + self.integral + derivative
        # output = self.bias
        
        self.last_position = vertical_ball_position
        return output
