#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
import roslib
roslib.load_manifest('swarm_node')
from swarm_node.msg import Health, Warn
import time

# node information
default_swarm_size = 10
sid, rid, ssz = -1, -1, -1 # node identification params
neighbors = [] # list of node neighbiors

"""
State machine class to detect faults, implements state machine
as desribed in the project report and website. The state machine
has two states, "waiting" and "running", takes a boolean as input
which denotes whether it has received a message from its neighbor
or not. it outputs a boolean which denotes whether a warning must
be issued or not.
"""
class StateMachine1(object):
    state = "waiting"
    T, Tmax = 0, 0
    neighbor_id = -1 # ID of the neighbor to which this machine corresponds
    def __init__(self, nid, t_max = 3.0):
        self.state = "waiting"
        self.Tmax = t_max
        self.neighbor_id = nid
    def update(self, msg):
        if self.state == "waiting":
            if msg == False:
                return False # no warning
            elif msg == True:
                self.state = "running"
                self.T = time.time()
                rospy.loginfo("%d received first message from %d", rid, self.neighbor_id)
                return False
        elif self.state == "running":
            if msg == True:
                self.T = time.time()
                return False
            elif time.time()-self.T > self.Tmax:
                return True
            else:
                return False


class StateMachine(object): # top level state machine for robot
    sm1 = {}
    def __init__(self, neighbor_list):
        for n in neighbors:
            self.sm1[n] = StateMachine1(nid=n)
    def update(self, msg):
        ret = []
        for n in self.sm1:
            warn = self.sm1[n].update(msg.robot_id == n)
            if warn:
                msg = create_warn_msg(n)
                ret.append(msg)
        return ret


def msg2str(msg):
    return 'msg: [sid={}, rid={}]'.format(msg.swarm_id, msg.robot_id)

def create_warn_msg(wid):
    msg = Warn()
    msg.swarm_id = sid
    msg.sender_id = rid
    msg.warn_id = wid
    return msg

def create_health_msg(sid, rid):
    msg = Health()
    msg.swarm_id = sid
    msg.robot_id = rid
    return msg

def get_node_params():
    global sid, rid, ssz, neighbors, sm

    sid = rospy.get_param('~swarm_id', 0)
    rid = rospy.get_param('~robot_id', -1)
    ssz = rospy.get_param('/swarm{}_size'.format(sid), default_swarm_size)
    neighbors = eval(rospy.get_param('~neighbors', []))
    for n in neighbors:
        assert (0 <= n and n < ssz), "Invalid neighbor ID [{}]".format(n)
    assert (sid >= 0),  "Swarm ID [{}] must be nonnegative".format(sid)
    assert (rid >= 0),  "Robot ID [{}] must be nonnegative".format(rid)
    assert (ssz > 0 ),  "Swarm size [{}] must be greater than 0".format(ssz)
    assert (rid < ssz), "Robot ID [{}] must be less than the swarm size [{}]".format(rid, ssz)

    sm = StateMachine(neighbors)
    return sid, rid, ssz, neighbors

def msg_callback(msg):
    if msg.robot_id in neighbors: # if the message is from a neighbor
        ret = sm.update(msg)
        for msg in ret:
            warn_pub.publish(msg)
            rospy.loginfo("Warning! %d did not receive message from %d", rid, msg.warn_id)

def main():
    global warn_pub

    rospy.init_node('swarm_node', anonymous=True)
    sid, rid, ssz, neighbors = get_node_params()
    rospy.loginfo('Created swarm node with [SID=%d, RID=%d, #neigbors=%d]', sid, rid, len(neighbors))

    warn_pub = rospy.Publisher('/warnings', Warn, queue_size=10)
    sub = rospy.Subscriber('/broadcast', Health, msg_callback)
    health_pub = rospy.Publisher('/broadcast', Health, queue_size=1)
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        msg = create_health_msg(sid, rid)
        health_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

