# Wheel Odometry: From Motor Speed to Position

## What we have
- `getMotorStatus2()` gives shaft speed in **deg/s**
- 6 motors: 3 left, 3 right (all same speed per side in differential drive)
- Wheel radius: 0.11 m
- Wheel base: 0.5 m

---

## Step 1: Shaft speed → wheel linear speed

Convert deg/s to rad/s, then to m/s:

```
wheel_vel (m/s) = shaft_speed_deg_s * (π / 180) * wheel_radius
```

For left and right sides, average the 3 motors per side:
```
left_vel  = avg(motor1, motor2, motor3) * (π/180) * 0.11
right_vel = avg(motor4, motor5, motor6) * (π/180) * 0.11
```

---

## Step 2: Differential drive → linear and angular velocity

```
linear_vel  = (right_vel + left_vel) / 2      # forward speed (m/s)
angular_vel = (right_vel - left_vel) / wheelbase  # yaw rate (rad/s)
```

These are the **derivatives** of position and heading.

---

## Step 3: Integrate velocity → position (the antiderivative)

We can't do continuous integration in code, so we do **discrete Euler integration** at each timestep dt:

```
x_new     = x + linear_vel * cos(theta) * dt
y_new     = y + linear_vel * sin(theta) * dt
theta_new = theta + angular_vel * dt
```

Where:
- `theta` is the current heading (yaw angle in radians)
- `dt` is time elapsed since last update (seconds)
- `cos(theta)` / `sin(theta)` project forward motion into x/y components

This is essentially a **Riemann sum** approximation of the integral:
$$x(t) = \int v(t)\cos\theta(t)\, dt$$
$$y(t) = \int v(t)\sin\theta(t)\, dt$$
$$\theta(t) = \int \omega(t)\, dt$$

The smaller dt is (higher frequency updates), the more accurate the integral.

---

## Step 4: Package into nav_msgs/Odometry

The EKF needs:
- `pose.pose.position.x/y` — from integrated x, y above
- `pose.pose.orientation` — quaternion from theta (use tf_transformations)
- `twist.twist.linear.x` — linear_vel
- `twist.twist.angular.z` — angular_vel
- Covariances — small but nonzero (e.g. 0.01 on diagonal)

The EKF will fuse this with the IMU to smooth out wheel slip errors.

---

## Key caveat
Wheel odometry **drifts over time** because errors accumulate in the integral.
That's exactly why we use an EKF — the IMU catches angle drift, and eventually
GPS (or ZED VIO) can correct the position drift.
