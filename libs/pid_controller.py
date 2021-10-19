import numpy as np

from libs.normalise_angle import normalise_angle

class PIDTracker:

    def __init__(self, wheelbase, steering_limits, dt):

        self.steering_limits = steering_limits
        self.wheelbase = wheelbase
        self.dt = dt
        
        self.Kp = 0.01
        self.Ki = 0.01
        self.Kd = 0.01

    def target_index_calculator(self, x, y, yaw, path_x, path_y, path_yaw):

        # Calculate position of the front axle
        fx = x + self.wheelbase * np.cos(yaw)
        fy = y + self.wheelbase * np.sin(yaw)

        dx = fx - path_x    # Find the x-axis of the front axle relative to the path
        dy = fy - path_y    # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy)       # Find the distance from the front axle to the path
        target_idx = np.argmin(d)  # Find the shortest distance in the array

        # Heading error
        heading_term = normalise_angle(path_yaw[target_idx] - yaw)

        return heading_term

    def proportional_control(self, error):
        
        return self.Kp * error

    def integral_control(self, error):
        
        return self.Ki * error * self.dt

    def derivative_control(self, error, previous_error):
        
        return self.Kd * (error - previous_error)/self.dt

    def heading_control(self, delta, x, y, yaw, path_x, path_y, path_yaw, prev_heading):
            
        heading_term = self.target_index_calculator(x, y, yaw, path_x, path_y, path_yaw)

        mv_p = self.proportional_control(heading_term)
        mv_i = self.integral_control(heading_term)
        mv_d = self.derivative_control(heading_term, prev_heading)

        sigma = delta + mv_p + mv_i + mv_d

        if sigma >= self.steering_limits:
                sigma = self.steering_limits

        elif sigma <= -self.steering_limits:
            sigma = -self.steering_limits

        return sigma

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()