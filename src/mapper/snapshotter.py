#!/usr/bin/env python
# todo: license here

import rospy
import time

from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2

def pose_est():
    rospy.init_node('periodic_laser_snapshotter')

    rate = 1.0/5.0

    rospy.wait_for_service('build_cloud')

    assemble = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    
    pub = rospy.Publisher('pointcloud', PointCloud2, queue_size=10)

    # start running at rate
    last = rospy.get_rostime() # make a cumulative point cloud
    time.sleep(5) 
    next = time.time()
    while not rospy.is_shutdown():
        resp = assemble(last, rospy.get_rostime())
        # last = rospy.get_rostime() # make a cumulative point cloud
        pub.publish(resp.cloud)

        next = next + 1.0/rate
        try:
            time.sleep(next-time.time())
        except rospy.ROSInterruptException:
            return
        except:
            # if timing is off, just skip
            pass

    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        pose_est()
    except rospy.ROSInterruptException:
        pass
