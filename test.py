#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-
import roslib; roslib.load_manifest('entres')
import rospy
import entres.srv as srv
import os, random, string
from table_game import TableGame

class ResultCopy:
  def __init__ (self, other):
    self.params = other.params
    self.orig_params = list(other.params)
    self.probability = other.probability
    self.functionName = other.functionName

class Client:
  def __init__(self):
    self.model = "er-persists-actions-learned.mln"
    self.queries = ["is","explainOld","explainNew"]
    self.success = 0
    self.fail = 0
    self.failed = []
    self.total = 0

  def connect (self):
    rospy.wait_for_service('entres_server')
    self.infer = rospy.ServiceProxy('entres_server', srv.Infer)

  def save_train (self, state, filename):
    if len(state.old_clusters) < 1 or len(state.new_clusters) < 1:
      return
    with open (filename, "w") as f:
      # object/cluster domains
      f.write ("// old: %s\n" % repr(state.old_clusters))
      f.write ("// new: %s\n" % repr(state.new_clusters))
      f.write ("// persisting: %s\n" % repr(state.persisting_clusters))
      f.write ("// outofview: %s\n" % repr(state.out_of_view))
      f.write ("//\n// truth: %s\n" % state.truth)


      f.write ("\n")

      f.write ("oldClusters={%s}\n" % ",".join([o.oldName() for o in state.old_clusters]))
      f.write ("newClusters={%s}\n" % ",".join([o.newName() for o in state.new_clusters]))
      f.write ("\n")

      # similarity evidence
      for n in state.new_clusters:
        for o in state.old_clusters:
          f.write ("%f looksSimilar(%s,%s)\n" % (o.similarApp(n), o.oldName(), n.newName()))
          f.write ("%f samePos(%s,%s)\n" % (o.similarPos(n), o.oldName(), n.newName()))
      f.write ("\n")

      for p in state.persisting_clusters:
        f.write("isPersisting(%s)\n" % p.oldName())

      for oov in state.out_of_view:
        f.write("outOfView(%s)\n" % state.old_clusters[oov].oldName())

      f.write("\n")
      # ground truth
      for n in state.new_clusters:
        for o in state.old_clusters:
          if o.name == n.name:
            f.write ("is(%s,%s)\n" % (o.oldName(), n.newName()))
          else:
            f.write ("!is(%s,%s)\n" % (o.oldName(), n.newName()))

      f.write("\n")
      for o in state.old_clusters:
        if o.truth == "s":
          f.write ("explainOld(%s,STAY)\n" % o.oldName())
        elif o.truth == "m":
          f.write ("explainOld(%s,MOVE)\n" % o.oldName())
        elif o.truth == "d":
          f.write ("explainOld(%s,DISAPPEAR)\n" % o.oldName())
        elif o.truth == "p":
          f.write ("explainOld(%s,PERSIST)\n" % o.oldName())
        elif o.truth == "v":
          f.write ("explainOld(%s,NEWVIEW)\n" % o.oldName())
        else:
          print "holy fuck:"
          print o
          print state
          import sys
          sys.exit ()
          

      f.write("\n")
      for n in state.new_clusters:
        if n.truth == "s":
          f.write ("explainNew(%s,STAY)\n" % n.newName())
        elif n.truth == "m":
          f.write ("explainNew(%s,MOVE)\n" % n.newName())
        elif n.truth == "a":
          f.write ("explainNew(%s,APPEAR)\n" % n.newName())
        elif n.truth == "v":
          f.write ("explainNew(%s,NEWVIEW)\n" % n.newName())


  def call (self, state):
    try:
      response = self.infer(
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
              preprefix = "ok  \t"
            else:
              preprefix = "  no\t"
          else:
            if gndF.probability < 0.25:
              preprefix = "ok  \t"
            else:
              preprefix = "  no\t"
          
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


def test():
  c = Client ()
  c.connect ()

  s = TableGame()
  nr_train = 10
  for i in range(nr_train):
    s.do_time_step ()
    state = s.get_table_state ()

    c.call(state)
  print "success:", c.success
  print "fail:", c.fail
  print "total:", c.total

  with open("failed", "w") as f:
    f.write (c.failed.__repr__())

def train():
  c = Client ()

  basename = "".join(random.choice (string.ascii_lowercase) for _ in range (6)) + "_"

  s = TableGame()
  last_p = -1
  nr_train = 10000
  for i in range(nr_train):
    p = int(100.0 * float(i) / float(nr_train))
    if p != last_p:
      last_p = p
      print "%i %%" % p
    s.do_time_step ()
    state = s.get_table_state ()
    filename = os.path.join("./train4", basename + ("%05i.db" % i))
    c.save_train(state, filename)
  print


if __name__ == "__main__":
  train()
  #test()

