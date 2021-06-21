import os
import matplotlib.pyplot as plt
import numpy as np

from velocity_control import velocity_control
from scipy import signal
from matplotlib.animation import FuncAnimation
from libs.stanley_controller import PathTracker
from libs.car_description import Description
from libs.kinematic_model import KinematicBicycleModel

class Simulation:

    def __init__(self):

        fps = 50.0

        self.dt = 1/fps
        self.map_size = 20
        self.frames = 1500
        self.loop = False

class Path:

    def __init__(self):

        self.px = np.linspace(0, 1500, num=1500)
        self.py = [0 for i in range(1500)]
        self.pyaw = [0 for i in range(1500)]


class TargetCar:

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y 
        self.yaw = init_yaw
        self.v = 0.0
        self.throttle = 0.0
        self.delta = 0.0
        self.omega = 0.0
        self.L = 2.5
        self.max_steer = np.deg2rad(33)
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 10.0
        self.ksoft = 10.0
        self.xtrackerr = None
        self.target_id = None

        # Description parameters
        self.length = 4.5
        self.width = 2.0
        self.rear2wheel = 1.0
        self.wheel_dia = 0.15 * 2
        self.wheel_width = 0.2
        self.tread = 0.7

    def drive(self):
        
        self.tracker = PathTracker(self.k, self.ksoft, self.max_steer, self.L, self.throttle, self.x, self.y, self.yaw, self.px, self.py, self.pyaw)
        self.throttle, self.delta = self.tracker.stanley_control()
        self.kbm = KinematicBicycleModel(self.x, self.y, self.yaw, self.v, self.throttle, self.delta, self.L, self.max_steer, self.dt)
        self.x, self.y, self.yaw, self.v, self.delta, self.omega = self.kbm.kinematic_model()

class Car:

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y 
        self.yaw = init_yaw
        self.v = 0.0
        self.throttle = 100
        self.delta = 0.0
        self.omega = 0.0
        self.L = 2.5
        self.max_steer = np.deg2rad(33)
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 10.0
        self.ksoft = 30.0

        # Description parameters
        self.length = 4.5
        self.width = 2.0
        self.rear2wheel = 1.0
        self.wheel_dia = 0.15 * 2
        self.wheel_width = 0.2
        self.tread = 0.7

        # Velocity control parameters
        self.gap = 0.0
        self.prev_gap = 0.0
        self.safety_thresh = 10.0

    def drive(self, target_x, target_y):
        
        self.gap = np.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        self.throttle = velocity_control(self.throttle, self.gap, self.prev_gap, self.safety_thresh, self.dt)
        self.tracker = PathTracker(self.k, self.ksoft, self.max_steer, self.L, self.throttle, self.x, self.y, self.yaw, self.px, self.py, self.pyaw)
        self.throttle, self.delta = self.tracker.stanley_control()
        self.kbm = KinematicBicycleModel(self.x, self.y, self.yaw, self.v, self.throttle, self.delta, self.L, self.max_steer, self.dt)
        self.x, self.y, self.yaw, self.v, self.delta, self.omega = self.kbm.kinematic_model()
        self.prev_gap = self.gap

        os.system('cls' if os.name=='nt' else 'clear')

def main():
    
    sim = Simulation()
    path = Path()
    car = Car(path.px[0], path.py[0], path.pyaw[0], sim, path)
    target = TargetCar(30, path.py[0], path.pyaw[0], sim, path)
    car_desc = Description(car.length, car.width, car.rear2wheel, car.wheel_dia, car.wheel_width, car.tread, car.L)
    target_desc = Description(target.length, target.width, target.rear2wheel, target.wheel_dia, target.wheel_width, target.tread, target.L)

    interval = sim.dt * 10**3

    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    ax[0].set_aspect('equal')

    ax[0].axhline(2)
    ax[0].axhline(-2)

    annotation_car = ax[0].annotate('{}'.format(np.around(car.gap, 2)), xy=(car.x, car.y + 5), color='black', annotation_clip=False)

    car_color = 'green'
    outline, = ax[0].plot([], [], color=car_color)
    fr, = ax[0].plot([], [], color=car_color)
    rr, = ax[0].plot([], [], color=car_color)
    fl, = ax[0].plot([], [], color=car_color)
    rl, = ax[0].plot([], [], color=car_color)
    rear_axle, = ax[0].plot(car.x, car.y, '+', color='black', markersize=2)

    annotation_target = ax[0].annotate('{}'.format(np.around(target.v, 2)), xy=(target.x, target.y + 5), color='black', annotation_clip=False)

    target_color = 'red'
    outline_t, = ax[0].plot([], [], color=target_color)
    fr_t, = ax[0].plot([], [], color=target_color)
    rr_t, = ax[0].plot([], [], color=target_color)
    fl_t, = ax[0].plot([], [], color=target_color)
    rl_t, = ax[0].plot([], [], color=target_color)
    rear_axle_t, = ax[0].plot(car.x, car.y, '+', color='black', markersize=2)

    # target_vel = (np.abs(signal.sawtooth(2*np.pi*5*path.px, 0.5)) + 1) *20
    target_vel = np.flip(signal.sawtooth(2*np.pi*5*path.px, 0.5))*20

    ax[1].axhline(car.safety_thresh, color='red')
    gap_data, = ax[1].plot([], [])
    ax[1].set_xlim(0, sim.frames)
    gap_arr = []
    frames = []

    ax[0].grid()
    ax[1].grid()

    def animate(frame):
        # Camera tracks car
        ax[0].set_xlim(car.x - sim.map_size, car.x + sim.map_size)
        ax[0].set_ylim(-4, 4)

        # Drive and draw main car
        car.drive(target.x, target.y)
        outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = car_desc.plot_car(car.x, car.y, car.yaw, car.delta)
        outline.set_data(outline_plot[0], outline_plot[1])
        fr.set_data(fr_plot[0], fr_plot[1])
        rr.set_data(rr_plot[0], rr_plot[1])
        fl.set_data(fl_plot[0], fl_plot[1])
        rl.set_data(rl_plot[0], rl_plot[1])
        rear_axle.set_data(car.x, car.y)

        # Drive and draw target car
        target.v = target_vel[frame]
        target.drive()
        outline_plot_t, fr_plot_t, rr_plot_t, fl_plot_t, rl_plot_t = target_desc.plot_car(target.x, target.y, target.yaw, target.delta)
        outline_t.set_data(outline_plot_t[0], outline_plot_t[1])
        fr_t.set_data(fr_plot_t[0], fr_plot_t[1])
        rr_t.set_data(rr_plot_t[0], rr_plot_t[1])
        fl_t.set_data(fl_plot_t[0], fl_plot_t[1])
        rl_t.set_data(rl_plot_t[0], rl_plot_t[1])
        rear_axle_t.set_data(target.x, target.y)

        # Annotate gap between cars above car
        annotation_car.set_text('Gap: {} m'.format(np.around(car.gap, 2)))
        annotation_car.set_position((car.x, car.y + 3))

        # Annotate velocity of target
        annotation_target.set_text('Velocity: {} m/s'.format(np.around(target.v, 2)))
        annotation_target.set_position((target.x, target.y + 3))

        # Animate graph
        gap_arr.append(car.gap)
        frames.append(frame)
        ax[1].set_ylim(gap_arr[-1] - sim.map_size, gap_arr[-1] + sim.map_size)
        gap_data.set_data(frames, gap_arr)

        ax[0].set_title('{}s'.format(np.around(sim.dt * frame, 2)), loc='right')
        ax[0].set_xlabel('Speed: {} m/s'.format(np.around(car.v, 2)), loc='left')

        return outline, fr, rr, fl, rl, rear_axle, outline_t, fr_t, rr_t, fl_t, rl_t, rear_axle_t, gap_data,

    anim = FuncAnimation(fig, animate, frames=sim.frames, interval=interval, repeat=sim.loop)
    anim.save('animation.gif', writer='imagemagick', fps=50)
    plt.show()

if __name__ == '__main__':
    main()