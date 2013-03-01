#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-
import roslib; roslib.load_manifest('entres')
import rospy
import entres.srv as srv

rospy.wait_for_service('entres_server')
infer = rospy.ServiceProxy('entres_server', srv.Infer)
try:
  response = infer(
      model="er-persists-actions.mln",
      num_old_clusters = 2,
      num_new_clusters = 3,
      similarAppearance = [0.0,0.0,
                           0.0,1.0,
                           0.0,0.0],
      similarPose = [1.0,0.0,
                     0.0,1.0,
                     0.0,0.0],
      out_of_view = [1,2],
      queries=["is","explainOld","explainNew"]
      )
  
  for gndF in response.results:
    params_string = ",".join(gndF.params)
    print "%s(%s) : %f" % (gndF.functionName, params_string, gndF.probability)

  print "persistent clusters:", response.persistent
  print "new-to-old mappings:", response.new_to_old

except rospy.ServiceException, e:
  print "Service did not process request: %s" % str(e)
