# Data-synchronization-and-collection
After you install this ros package, in the `/data` folder (should be created by yourself anywhere), run:

```
rosrun data_for_deepvio sync_subscriber_node
```
Then move jackal in the gazebo, and type `ctrl+c` to finish the collection
It will give you synchronized image data, odometry data and imu data (now at 4hz), and will be 10 imu data between two synchronized data(40 hz).
