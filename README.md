# Collection of ROS Utilities

## Synchronized Stereo Image Saver
Will read a bag file and save the an image topic to file for stereo pair.
Also supports compressed image stream.

```
roslaunch kros_utils stereo_saver.launch
```

## Compressed Image Topic to Uncompressed

```
rosrun image_transport republish compressed in:=/camera/fisheye1/image_raw _image_transport:=compressed  out:=/out/camera/fisheye1/image_raw/
```

## Unsplit Lidar Cloud
Converts the split lidar cloud to unsplit cloud. The split cloud has a
message at every acquisition. It is often at very high framerate (~1500 Hz).
The script accumulates several clouds and publishes at a lower rate
say 10 Hz.

```
roslaunch kros_utils unsplit_lidar_ptcld.launch
```

# Author
Manohar Kuse <mpkuse@connect.ust.hk>
