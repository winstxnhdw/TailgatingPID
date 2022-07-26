# TailgatingPID

[![Total alerts](https://img.shields.io/lgtm/alerts/g/winstxnhdw/TailgatingPID.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/winstxnhdw/TailgatingPID/alerts/)
<a href="https://lgtm.com/projects/g/winstxnhdw/TailgatingPID/alerts/"><img alt="Total alerts" src="https://img.shields.io/lgtm/alerts/g/winstxnhdw/TailgatingPID.svg?logo=lgtm&logoWidth=18"/></a>

Experimental velocity control based on a simple PID controller. The line plot illustrates the change in distance between the two vehicles in every frame. The following two animations are the simulated experiments.

```yaml
:param accel:               (float) vehicle's current velocity [m/s]
:param max_accel:           (float) vehicle's maximum accleration [m/s^2]
:param gap:                 (float) vehicle's vehicle from the target [m]
:param prev_gap:            (float) previous vehicle's distance from the target [m]
:param safety_thresh:       (float) maximum allowable distance from the target [m]
:param dt:                  (float) discrete time period [s]

:return desired_accel:      (float) vehicle's desired velocity [m/s]
```
## Installation
> To run the animation, if not just install Numpy.
```bash
$ pip install -r requirements.txt
```

## The Police Chase
<div align="center">
	<img src="resources/police_chase.gif" />
</div>

## The Girlfriend
<div align="center">
	<img src="resources/the_girlfriend.gif" />
</div>
