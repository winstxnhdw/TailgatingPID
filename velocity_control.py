import numpy as np

class VelocityController:

    def __init__(self, Kp=30, Ki=1, Kd=25):
        '''
        :param accel:               (float) vehicle's current velocity [m/s]
        :param max_accel:           (float) vehicle's maximum accleration [m/s^2]
        :param gap:                 (float) vehicle's vehicle from the target [m]
        :param prev_gap:            (float) previous vehicle's distance from the target [m]
        :param safety_thresh:       (float) maximum allowable distance from the target [m]
        :param dt:                  (float) discrete time period [s]

        :return desired_accel:      (float) vehicle's desired velocity [m/s]
        '''

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def proportional_control(self, error):
        
        return self.Kp * error

    def integral_control(self, error, dt):
        
        return self.Ki * error * dt

    def derivative_control(self, error, previous_error, dt):
        
        return self.Kd * (error - previous_error)/dt

def velocity_control(accel, max_accel, gap, prev_gap, safety_thresh, dt):

    controller = VelocityController()

    err = gap - safety_thresh
    prev_err = prev_gap - safety_thresh

    mv_p = controller.proportional_control(err)
    mv_i = controller.integral_control(err, dt)
    mv_d = controller.derivative_control(err, prev_err, dt)

    desired_accel = np.clip(accel + mv_p + mv_i + mv_d, -max_accel, max_accel)

    print(f"Desired throttle: {desired_accel} m/s^2")
    print(f"Error: {err} m".format(err))
    print(f"Proportional control: {mv_p} m/s^2")
    print(f"Integral control: {mv_i} m/s^2")
    print(f"Derivative control: {mv_d} m/s^2")

    return desired_accel

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()
