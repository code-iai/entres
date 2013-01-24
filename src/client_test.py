# -*- coding: iso-8859-1 -*-
import roslib; roslib.load_manifest('probcog')
import rospy
import probcog.srv as srv

rospy.wait_for_service('probcog_infer')
infer = rospy.ServiceProxy('probcog_infer', srv.Infer)
try:
  response = infer(model="meals_bln", evidence=["takesPartIn(P,M)"], queries=["consumesAnyIn"])
  print response
except rospy.ServiceException, e:
  print "Service did not process request: %s" % str(e)