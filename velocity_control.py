class VelocityController:

    def __init__(self):
        '''
        :param v:               (float) vehicle's current velocity [m/s]
        :param gap:             (float) vehicle's vehicle from the target [m]
        :param prev_gap:        (float) previous vehicle's distance from the target [m]
        :param safety_thresh:   (float) maximum allowable distance from the target [m]
        :param dt:              (float) discrete time period [s]

        :return desired_accel:    (float) vehicle's desired velocity [m/s]
        '''

        self.Kp = 1
        self.Ki = 0
        self.Kd = 0

    def proportional_control(self, error):
        
        return self.Kp * error

    def integral_control(self, error, dt):
        
        return self.Ki * error * dt

    def derivative_control(self, error, previous_error, dt):
        
        return self.Kd * (error - previous_error)/dt

def velocity_control(v, gap, prev_gap, safety_thresh, dt):

    controller = VelocityController()

    err = gap - safety_thresh
    prev_err = prev_gap - safety_thresh

    mv_p = controller.proportional_control(err)
    mv_i = controller.integral_control(err, dt)
    mv_d = controller.derivative_control(err, prev_err, dt)

    desired_accel = v + mv_p + mv_i + mv_d

    if desired_accel < 0:
        desired_accel = 0

    print("Desired throttle: {} m/s^2".format(desired_accel))

    return desired_accel

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()