# To use for v2 laser

rosrun topic_tools relay_field /lidar/scan_fused /lidar/scan_slam sensor_msgs/LaserScan "header:
  seq: m.header.seq
  stamp:
    secs: m.header.stamp.secs
    nsecs: m.header.stamp.nsecs
  frame_id: m.header.frame_id
angle_min: m.angle_min
angle_max: m.angle_max
angle_increment: 0.00435727136
time_increment: m.time_increment
scan_time: m.scan_time
range_min: m.range_min
range_max: m.range_max
ranges: m.ranges
intensities: m.intensities
"
