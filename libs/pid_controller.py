import numpy as np

from libs.normalise_angle import normalise_angle

class PIDTracker:

    def __init__(self):
        
        self.Kp = 0.01
        self.Ki = 0.01
        self.Kd = 0.01

    def target_index_calculator(self, x, y, yaw, path_x, path_y, path_yaw, wheelbase):

        # Calculate position of the front axle
        fx = x + wheelbase * np.cos(yaw)
        fy = y + wheelbase * np.sin(yaw)

        dx = [fx - icx for icx in path_x] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in path_y] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Heading error
        heading_term = normalise_angle(path_yaw[target_idx] - yaw)

        return heading_term

    def proportional_control(self, error):
        
        return self.Kp * error

    def integral_control(self, error, dt):
        
        return self.Ki * error * dt

    def derivative_control(self, error, previous_error, dt):
        
        return self.Kd * (error - previous_error)/dt

def pid_control(delta, steering_limits, wheelbase, x, y, yaw, path_x, path_y, path_yaw, prev_heading, dt):
        
    tracker = PIDTracker()

    heading_term = tracker.target_index_calculator(x, y, yaw, path_x, path_y, path_yaw, wheelbase)

    mv_p = tracker.proportional_control(heading_term)
    mv_i = tracker.integral_control(heading_term, dt)
    mv_d = tracker.derivative_control(heading_term, prev_heading, dt)

    sigma = delta + mv_p + mv_i + mv_d

    if sigma >= steering_limits:
            sigma = steering_limits

    elif sigma <= -steering_limits:
        sigma = -steering_limits

    return sigma

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()