#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
import roslib
roslib.load_manifest('swarm_node')
from swarm_node.msg import Health

# node information
default_swarm_size = 10
sid, gid, rid, ssz = -1, -1, -1, -1

def msg2str(msg):
    return 'msg: [sid={}, gid={}, rid={}]'.format(msg.swarm_id, msg.group_id, msg.robot_id)

def create_health_msg(sid, gid, rid):
    msg = Health()
    msg.swarm_id = sid
    msg.group_id = gid
    msg.robot_id = rid
    return msg

def get_node_params():
    global sid, gid, rid, ssz

    sid = rospy.get_param('~swarm_id', -1)
    gid = rospy.get_param('~group_id', -1)
    rid = rospy.get_param('~robot_id', -1)
    ssz = rospy.get_param('/swarm{}_size'.format(sid), default_swarm_size)
    assert (sid >= 0),  "Swarm ID [{}] must be nonnegative".format(sid)
    assert (gid >= 0),  "Group ID [{}] must be nonnegative".format(gid)
    assert (rid >= 0),  "Robot ID [{}] must be nonnegative".format(rid)
    assert (ssz > 0 ),  "Swarm size [{}] must be greater than 0".format(ssz)
    assert (rid < ssz), "Robot ID [{}] must be less than the swarm size [{}]".format(rid, ssz)

    return sid, gid, rid, ssz

def msg_callback(msg):
    # do something with the message

def main():
    # connect to ros master
    rospy.init_node('swarm_node', anonymous=True)

    # get node parameters
    sid, gid, rid, ssz = get_node_params()
    rospy.loginfo('Created swarm node with [SID=%d, GID=%d, RID=%d]', sid, gid, rid)

    # listen to broadcast
    sub = rospy.Subscriber('/broadcast', Health, msg_callback)

    # start broadcast
    pub = rospy.Publisher('/broadcast', Health, queue_size=1)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        msg = create_health_msg(sid, gid, rid)
        rospy.loginfo(msg2str(msg))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
