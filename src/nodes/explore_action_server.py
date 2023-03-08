#! /usr/bin/env python3

import roslib
roslib.load_manifest('queenie')
import rospy
import actionlib

from explore import ExploreObject
from queenie.msg import ExploreAction, ExploreResult

class ExploreServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('explore', ExploreAction, self.execute, False)
    self.explore_controller = ExploreObject()
    self.server.start()
    print("explore action server has been successfully started")

  def execute(self, goal):
    result = ExploreResult()
    success = self.explore_controller.explore()
    if success:
        result.handleInSight = True
    else:
        result.handleInSight = False
        
    self.server.set_succeeded(result)


if __name__ == '__main__':
  rospy.init_node('explore_server')
  server = ExploreServer()
  rospy.spin()
