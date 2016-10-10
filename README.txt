Swarm Node
==========

This package provides a node which can be used to turn a simple
robot into a part of a large swarm of robots. It registers the
robot in the swarm, assigns it a task and monitors its health to
check if it needs to be substituted.

published topics:
-----------------
/broadcast

subscribed topics:
------------------
/broadcast

parameters:
-----------
~swarm_id (required)
~robot_id (required)
/swarm_size (optional, default: 10)

