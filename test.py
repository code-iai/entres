#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-
import roslib; roslib.load_manifest('entres')
import rospy
import entres.srv as srv
from table_game import TableGame

class ResultCopy:
  def __init__ (self, other):
    self.params = other.params
    self.orig_params = list(other.params)
    self.probability = other.probability
    self.functionName = other.functionName

class Client:
  def __init__(self):
    self.model = "er-persists-actions.mln"
    self.queries = ["is","explainOld","explainNew"]
    self.success = 0
    self.fail = 0
    self.failed = []
    self.total = 0

  def call (self, state):
    try:
      response = infer(
          model=self.model,
          num_old_clusters = len(state.old_clusters),
          num_new_clusters = len(state.new_clusters),
          similarAppearance = state.similar_appearance,
          similarPose = state.similar_pose,
          out_of_view = state.out_of_view,
          queries=self.queries
        )
      
      old_fail = self.fail

      results_copy = [ResultCopy(gndF) for gndF in response.results]
      
      for gndF in results_copy:
        for i,p in enumerate(gndF.params):
          if p[0] == "N" and "0" <= p[1:2] <= "9":
            idx = int(p[1:])
            gndF.params[i] = repr(state.new_clusters[idx])
          if p[0] == "O" and "0" <= p[1:2] <= "9":
            idx = int(p[1:])
            gndF.params[i] = repr(state.old_clusters[idx])

        params_string = ",".join(gndF.params)
        if gndF.functionName == "is":
          if gndF.params[0] == gndF.params[1]:
            self.total += 1
            if gndF.probability > 0.75:
              self.success += 1
            else:
              self.fail += 1
              with open("failed", "a") as f:
                f.write (gndF.params[0] + "\n")

        preprefix = ""
        prefix = ""
        if gndF.functionName == "is":
          idxO = int(gndF.orig_params[0][1:])
          idxN = int(gndF.orig_params[1][1:])
          idx = idxN * len(state.old_clusters) + idxO
          prefix = "(" + str(state.similar_appearance[idx]) + ")"
          prefix += " (" + str(state.similar_pose[idx]) + ")"
          if gndF.params[0] == gndF.params[1]:
            if gndF.probability > 0.75:
              preprefix = "ok\t"
            else:
              preprefix = "no\t"
          else:
            if gndF.probability < 0.25:
              preprefix = "ok\t"
            else:
              preprefix = "no\t"
          
        print "%s%s\t%s(%s) : %f" % (preprefix, prefix, gndF.functionName, params_string, gndF.probability)

      if old_fail != self.fail:
        with open("failed", "a") as f:
          for gndF in results_copy:
            params_string = ",".join(gndF.params)
            f.write ("%s(%s) : %f\n" % (gndF.functionName, params_string, gndF.probability))

      print "persistent clusters:", response.persistent
      print "new-to-old mappings:", response.new_to_old

    except rospy.ServiceException, e:
      print "Service did not process request: %s" % str(e)

if __name__ == "__main__":
  rospy.wait_for_service('entres_server')
  infer = rospy.ServiceProxy('entres_server', srv.Infer)
  c = Client ()


  s = TableGame()
  for i in range(10):
    s.do_time_step ()
    state = s.get_table_state ()
    print "-----", state
    c.call(state)
  print "success:", c.success
  print "fail:", c.fail
  print "total:", c.total

  with open("failed", "w") as f:
    f.write (c.failed.__repr__())
