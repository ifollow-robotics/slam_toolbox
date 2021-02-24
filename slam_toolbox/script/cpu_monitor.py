#!/usr/bin/env python
# license removed for brevity
import rospy
import psutil
from std_msgs.msg import Float32

def talker():
    cpu_pub = rospy.Publisher('cpu_perc', Float32, queue_size=1)
    mem_pub = rospy.Publisher('mem_perc', Float32, queue_size=1)
    rospy.init_node('cpu_monitor_for_loc', anonymous=True)
    rate = rospy.Rate(3)
    p_list = []

    for i in psutil.process_iter(attrs=["pid","name"]):
        if 'slam_toolbox' in i.info['name'] or 'snap_map_icp' in i.info['name']:
            pid = i.info['pid']
            rospy.loginfo(pid)
            p = psutil.Process(pid)
            p_list.append(p)

    while not rospy.is_shutdown():
        cpu_sum = sum(p_i.cpu_percent(interval=1.0) for p_i in p_list)
        mem_sum = sum(p_i.memory_percent() for p_i in p_list)
        cpu_usage = Float32(cpu_sum)
        mem_usage = Float32(mem_sum)
        cpu_pub.publish(cpu_usage)
        mem_pub.publish(mem_usage)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
