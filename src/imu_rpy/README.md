# Using accelerometer and magnetometer read from pi to measure roll, pitch and yaw.
## Adafruit_LSM303 read >> accel (XYZ) + mag (X"Z"Y)
## Adafruit_L3GD20() read >> omega (XYZ) (in deg)

### imu_test_node.py: Using only accelerometer and magnetometer to measure RPY
### imu_filter.py: From imu_test_node.py, add average filter (0.98 now and 0.02 last one)
### imu_init.py: Initial yaw after start up
### imu_add_gyro.py: Different from the first one, also using gyro to measure R and P
angle = 0.98 * (angle + gyroData * dt) + 0.02 * (accData)

How to connect Adafruit 9 dof breakout and Pi?
[image](https://imgur.com/a/5qlMW)
