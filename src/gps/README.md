## Command
roslaunch gstar navsat2gpsfix.launch

rosrun gps_common utm_odometry_node

rosrun tf static_transform_publisher 0 0 0 0 0 map gps 10

## Loop Closure Test @ 3/7

### Path I traversed

![alt text](https://github.com/seanNCTU/my_pcl/tree/master/src/gps/image/path_traversed.png)

### Result
![alt text](https://github.com/seanNCTU/my_pcl/tree/master/src/gps/image/result.png)
