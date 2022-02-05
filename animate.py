import os
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy import signal
from velocity_control import velocity_control
from libs.stanley_controller import StanleyController
from libs.car_description import Description
from libs.kinematic_model import KinematicBicycleModel
from libs.pid_controller import PIDTracker

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
        self.py = [0] * 1500
        self.pyaw = [0] * 1500

class Car:
    
    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y 
        self.yaw = init_yaw
        self.velocity = 0.0
        self.throttle = 0.0
        self.delta = 0.0
        self.omega = 0.0
        self.wheelbase = 2.96
        self.max_steer = np.deg2rad(33)
        self.dt = sim_params.dt

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw

        # Description parameters
        self.overall_length = 4.97
        self.overall_width = 1.964
        self.tyre_diameter = 0.4826
        self.tyre_width = 0.2032
        self.axle_track = 1.662
        self.rear_overhang = (self.overall_length - self.wheelbase) / 2

        self.kbm = KinematicBicycleModel(self.wheelbase, self.max_steer, self.dt)

class TargetCar(Car):

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):
        
        super().__init__(init_x, init_y, init_yaw, sim_params, path_params)

        # Tracker parameters
        self.prev_delta = 0.0
        
        # Description parameters
        self.colour = 'red'

        self.pid = PIDTracker(self.wheelbase, self.max_steer, self.dt)

    def drive(self):
        
        self.delta = self.pid.heading_control(self.delta, self.x, self.y, self.yaw, self.px, self.py, self.pyaw, self.prev_delta)
        self.x, self.y, self.yaw, self.velocity, self.delta, self.omega = self.kbm.kinematic_model(self.x, self.y, self.yaw, self.velocity, self.throttle, self.delta)
        self.prev_delta = self.delta

    def behaviour(self, behaviour):

        if behaviour == 'police chase':
            return 20 * (np.abs(signal.sawtooth(2*np.pi*5*path.px, 0.5)) + 1)

        elif behaviour == 'girlfriend':
            return 20 * np.flip(signal.sawtooth(2*np.pi*5*path.px, 0.5))

        else:
            raise Exception("No such behaviour setting.")


class TailgatingCar(Car):

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):
        
        super().__init__(init_x, init_y, init_yaw, sim_params, path_params)

        # Tracker parameters
        self.k = 10.0
        self.ksoft = 30.0

        # Velocity control parameters
        self.max_accel = 30.0
        self.gap = 0.0
        self.prev_gap = 0.0
        self.safety_thresh = 10.0

        # Description parameters
        self.colour = 'green'

        self.tracker = StanleyController(self.k, self.ksoft, 0.0, 0.0, self.max_steer, self.wheelbase, self.px, self.py, self.pyaw)

    def drive(self, target_x, target_y):
        
        os.system('cls' if os.name=='nt' else 'clear')

        self.gap = np.hypot(target_x - self.x, target_y - self.y)

        self.throttle = velocity_control(self.throttle, self.max_accel, self.gap, self.prev_gap, self.safety_thresh, self.dt)
        self.delta = self.tracker.stanley_control(self.x, self.y, self.yaw, self.velocity, self.delta)
        self.x, self.y, self.yaw, self.velocity, self.delta, self.omega = self.kbm.kinematic_model(self.x, self.y, self.yaw, self.velocity, self.throttle, self.delta)

        self.prev_gap = self.gap

def main():
    
    sim = Simulation()
    path = Path()

    tailgate = TailgatingCar(path.px[0], path.py[0], path.pyaw[0], sim, path)
    target = TargetCar(30, path.py[0], path.pyaw[0], sim, path)
    car_desc = Description(tailgate.overall_length, tailgate.overall_width, tailgate.rear_overhang, tailgate.tyre_diameter, tailgate.tyre_width, tailgate.axle_track, tailgate.wheelbase)
    target_desc = Description(target.overall_length, target.overall_width, target.rear_overhang, target.tyre_diameter, target.tyre_width, target.axle_track, target.wheelbase)

    interval = sim.dt * 10**3

    # Plot settings
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    ax[0].set_aspect('equal')
    ax[0].axhline(2)
    ax[0].axhline(-2)
    ax[0].grid()
    ax[1].grid()

    # Tailgate settings
    annotation_tailgate = ax[0].annotate(f'{tailgate.gap:.2f}', xy=(tailgate.x, tailgate.y + 5), color='black', annotation_clip=False)
    tailgate_color = tailgate.colour
    outline, = ax[0].plot([], [], color=tailgate_color)
    fr, = ax[0].plot([], [], color=tailgate_color)
    rr, = ax[0].plot([], [], color=tailgate_color)
    fl, = ax[0].plot([], [], color=tailgate_color)
    rl, = ax[0].plot([], [], color=tailgate_color)
    rear_axle, = ax[0].plot(tailgate.x, tailgate.y, '+', color='black', markersize=2)

    # Target settings
    target_color = target.colour
    target_vel = target.behaviour("police chase")
    outline_t, = ax[0].plot([], [], color=target_color)
    fr_t, = ax[0].plot([], [], color=target_color)
    rr_t, = ax[0].plot([], [], color=target_color)
    fl_t, = ax[0].plot([], [], color=target_color)
    rl_t, = ax[0].plot([], [], color=target_color)
    rear_axle_t, = ax[0].plot(tailgate.x, tailgate.y, '+', color='black', markersize=2)
    annotation_target = ax[0].annotate(f'{target.velocity:.2f}', xy=(target.x, target.y + 5), color='black', annotation_clip=False)

    # Graph settings
    ax[1].axhline(tailgate.safety_thresh, color='red')
    ax[1].set_xlim(0, sim.frames)
    gap_data, = ax[1].plot([], [])
    gap_arr = []
    frames = []

    def animate(frame):
        
        # Camera tracks car
        ax[0].set_xlim(tailgate.x - sim.map_size, tailgate.x + sim.map_size)
        ax[0].set_ylim(-4, 4)

        # Drive and draw main car
        tailgate.drive(target.x, target.y)
        outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = car_desc.plot_car(tailgate.x, tailgate.y, tailgate.yaw, tailgate.delta)
        outline.set_data(outline_plot[0], outline_plot[1])
        fr.set_data(fr_plot[0], fr_plot[1])
        rr.set_data(rr_plot[0], rr_plot[1])
        fl.set_data(fl_plot[0], fl_plot[1])
        rl.set_data(rl_plot[0], rl_plot[1])
        rear_axle.set_data(tailgate.x, tailgate.y)

        # Drive and draw target car
        target.velocity = target_vel[frame]
        target.drive()
        outline_plot_t, fr_plot_t, rr_plot_t, fl_plot_t, rl_plot_t = target_desc.plot_car(target.x, target.y, target.yaw, target.delta)
        outline_t.set_data(outline_plot_t[0], outline_plot_t[1])
        fr_t.set_data(fr_plot_t[0], fr_plot_t[1])
        rr_t.set_data(rr_plot_t[0], rr_plot_t[1])
        fl_t.set_data(fl_plot_t[0], fl_plot_t[1])
        rl_t.set_data(rl_plot_t[0], rl_plot_t[1])
        rear_axle_t.set_data(target.x, target.y)

        # Annotate gap between cars above car
        annotation_tailgate.set_text(f"Gap: {tailgate.gap:.2f} m")
        annotation_tailgate.set_position((tailgate.x, tailgate.y + 3))

        # Annotate velocity of target
        annotation_target.set_text(f"Velocity: {target.velocity:.2f} m/s")
        annotation_target.set_position((target.x, target.y + 3))

        # Animate graph
        gap_arr.append(tailgate.gap)
        frames.append(frame)
        ax[1].set_ylim(gap_arr[-1] - sim.map_size, gap_arr[-1] + sim.map_size)
        gap_data.set_data(frames, gap_arr)

        ax[0].set_title(f'{sim.dt*frame:.2f}s', loc='right')
        ax[0].set_xlabel(f'Speed: {tailgate.velocity:.2f} m/s', loc='left')

        return outline, fr, rr, fl, rl, rear_axle, outline_t, fr_t, rr_t, fl_t, rl_t, rear_axle_t, gap_data,

    _ = FuncAnimation(fig, animate, frames=sim.frames, interval=interval, repeat=sim.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=50)

    plt.show()

if __name__ == '__main__':
    main()
