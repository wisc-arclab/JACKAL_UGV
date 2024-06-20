#Data-synchronization-and-collection
After you install this ros package, in the `/data` folder, run:

```
rosrun data_for_deepvio sync_subscriber_node
```
Then move jackal in the gazebo, and type `ctrl+c` to finish the collection
It will give you synchronized image data, odometry data and imu data, and imu data has more frequency than the other two data.
