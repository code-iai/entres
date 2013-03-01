#!/usr/bin/env python

import random, re, math, numpy
from time import time
import string

basename = "".join(random.choice (string.ascii_lowercase) for _ in range (6)) + "_"

from experiment_settings import *

class Logger:
  def __init__ (self, filename):
    self.filename = filename

  def info (self, what):
    with open (self.filename, "a") as f:
      f.write(what)

  def screen_info (self, what):
    with open (self.filename, "a") as f:
      f.write(what)
    print what,

logger = Logger('entres.log')

def eval_gauss (x, mu=0.0, sigma=1.0):
  y = 1 / ( math.sqrt(2. * math.pi * sigma) )  * math.exp( -((x - mu + 0.0) / sigma)**2 )
  return y

class Object:
  def __init__ (self, name):
    self.name = name
    self.create_appearance ()
    self.create_pose ()

  def oldName (self):
    return self.name.replace ("DB","O")

  def newName (self):
    return self.name.replace ("DB","N")

  def random_vec (self, size):
    return numpy.array([random.random() for _ in range(size)])

  def create_appearance (self):
    self.app = self.random_vec (5)

  def create_pose (self):
    self.pose = self.random_vec (2)

  def similarPos (self, other):
    norm = eval_gauss (0, sigma=pose_sigma)
    delta = numpy.linalg.norm(self.observePose() - other.observePose())
    return eval_gauss (delta, sigma=pose_sigma) / norm

  def observePose (self):
    return self.pose + (numpy.random.randn(2) * pose_noise)
    return self.pose # TODO: this would mean perfect, noiseless vision

  def observe (self):
    return self.app + (numpy.random.randn(5) * app_noise)
    return self.app # TODO: this would mean perfect, noiseless vision

  def similarApp (self, other):
    norm = eval_gauss (0, sigma=app_sigma)
    delta = numpy.linalg.norm(self.observe() - other.observe())
    return eval_gauss (delta, sigma=app_sigma) / norm

  def __repr__ (self):
    return self.name


class Salvatore:
  def __init__ (self):
    objects = [Object("DB%i"%i) for i in range(number_objects)]
    self.pdf = [(self.add_object, prob_add), 
                (self.remove_object, prob_remove), 
                (self.move_object, prob_move), 
                (self.do_nothing, prob_nothing)]

    self.cdf = [(i, sum(p for j,p in self.pdf if j < i)) for i,_ in self.pdf]

    self.newClusters = []
    self.oldClusters = []
    self.shelf = objects
    self.tables = [[] for _ in range(number_tables)]

    self.cur_iter = 0

  def add_object (self):
    if len(self.shelf) == 0:
      logger.info ("  XXX cannot add object (shelf empty)\n")
      return False
    if len(self.shelf) < number_objects - max_objects_on_table:
      logger.info ("  XXX cannot add object (too many objects)\n")
      return False
    
    obj = random.choice (self.shelf)
    self.shelf.remove (obj)
    table = random.randrange (0, number_tables) 
    logger.info ("adding object to table %i\n" % table)
    obj.create_pose ()
    self.tables[table].append (obj)
    logger.info ("action: ")
    logger.screen_info ("+%s "%obj.name)
    return True

  def random_table(self):
    return random.randrange (0, number_tables)

  def remove_object (self):
    table = random.randrange (0, number_tables) 
    if len(self.tables[table]) == 0:
      logger.info ("  XXX cannot remove object from table %i (empty)\n" % table)
      return False

    logger.info ("removing object from table %i\n" % table)
    obj = random.choice (self.tables[table])
    self.tables[table].remove(obj)
    self.shelf.append(obj)
    logger.info ("action: ")
    logger.screen_info ("-%s "%obj.name)
    return True

  def move_object (self):
    table_from = random.randrange (0, number_tables)
    table_to   = random.randrange (0, number_tables)

    if len(self.tables[table_from]) == 0:
      logger.info ("  XXX cannot move object from table %i (empty) to table %i\n" % (table_from, table_to))
      return False
    
    logger.info ("moving object from table %i to table %i\n" % (table_from, table_to))
    obj = random.choice (self.tables[table_from])
    self.tables[table_from].remove(obj)
    obj.create_pose ()
    self.tables[table_to].append (obj)
    logger.info ("action: ")
    logger.screen_info ("~%s "%obj.name)
    return True

  def do_nothing (self):
    logger.info ("doing nothing\n")
    logger.info ("action: ")
    logger.screen_info (". ")
    return True

  def select_action (self):
    R = max(i for r in [random.random()] for i,c in self.cdf if c <= r)
    return R

  def extract_persists (self, s):
    pattern = re.compile(r"""(?P<prob>.*?)\s*
                             persists\s*\(
                             (?P<cluster>.*?)\s*
                             \)\s*""", re.VERBOSE)
    match = pattern.match(s)

    prob = float (match.group("prob"))
    cluster = match.group("cluster")

    return (prob, cluster)

  def append_persisting_objects (self):
    s.results = None
    return
#TODO!!!
    for line in self.results:
      index = line.find ("persists")
      if index  >= 0:
        prob, cluster = self.extract_persists (line)
        if prob > 0.5:
          self.oldClusters.append (cluster)

  def do_time_step (self):
    self.cur_basename = basename + str(self.cur_iter)

    self.oldClusters = self.newClusters
    self.newClusters = []
    self.append_persisting_objects ()
    for _ in range (number_actions):
      done = False
      while not done:
        action = self.select_action ()
        if action:
          done = action ()
    table = random.randrange (0, number_tables) 
    self.observe_scene (table)
    self.create_evidence ()
    logger.screen_info ("(%i x %i) "%(len(self.oldClusters),len(self.newClusters)))

    from subprocess import Popen,PIPE
    if self.oldClusters and self.newClusters:
      with open (self.cur_basename + ".log", "w") as f:
        f.write (Popen("mlnquery -i er-persists.mln -q persists,gone,appeared,same,moved -e " + self.cur_basename + ".db -r " + self.cur_basename + ".results --run",
          stdout=PIPE, shell=True).stdout.read())
      self.results = open (self.cur_basename + ".results", "r").readlines()

    self.cur_iter += 1

  def create_evidence (self):
    eA = []
    eP = []
    for o in self.oldClusters:
      for n in self.newClusters:
        eA.append ("%f similarApp(%s,%s)"%(o.similarApp(n), o.oldName(), n.newName()))
        eP.append ("%f similarPos(%s,%s)"%(o.similarPos(n), o.oldName(), n.newName()))
    self.evidence = eA + [""] + eP
    with open(self.cur_basename + ".db", "w") as f:
      oc = "oldClusters = {%s}" % (",".join([o.oldName() for o in self.oldClusters]))
      nc = "newClusters = {%s}" % (",".join([o.newName() for o in self.newClusters]))
      f.write (oc + "\n")
      f.write (nc + "\n")
      f.write ("\n")
      for e in self.evidence:
        f.write (e + "\n")

  def observe_scene (self, table):
    self.newClusters = []
    for t in range(len(self.tables)):
      if t == table:
        for obj in self.tables[t]:
          self.newClusters.append (obj)

"""The Oracle tries to make sense of the MLN inference results"""
class Oracle:
  def __init__ (self):
    regex = r"""(?P<prob>.*?)\s\s*
                (?P<pred>.*?)\s*\(
                (?P<op>.*?)\s*
                \)\s*"""

    self.pattern = re.compile (regex, re.VERBOSE)
    self.results = {}

  def parse_line (self, line):
    match = self.pattern.match (line)

    prob = float (match.group("prob"))
    pred = match.group("pred")
    ops = match.group("op").split(",")

    return prob, pred, ops

  def read (self, results):
    resultDict = {}
    for line in results:
      prob, pred, ops = self.parse_line (line)
      if len(ops) == 1:
        if not resultDict.has_key (ops[0]):
          resultDict[ops[0]] = []
        resultDict[ops[0]].append ({"pred": pred, "prob":prob})
      elif len(ops) == 2:
        if not resultDict.has_key (ops[0]):
          resultDict[ops[0]] = []
        resultDict[ops[0]].append ({"pred": pred, "prob":prob, "op":ops[1]})
        if not resultDict.has_key (ops[1]):
          resultDict[ops[1]] = []
        resultDict[ops[1]].append ({"pred": pred, "prob":prob, "op":ops[0]})
      else:
        logger.screen_info ("Fuck, something went wrong: %s %s %s \n"%(prob, pred, ops))
    self.results = resultDict
    return self

  def interpret (self):
    for key in self.results.keys():
      if key[0] == "N":
        p = self.results[key]
        p.sort (key=lambda x: x["prob"], reverse=True)
        if p[0]["pred"] == "appeared":
          logger.info ("reconstructed: +%s\n"%key)
        if p[0]["pred"] == "moved":
          logger.info ("reconstructed: ~%s\n"%key)
        logger.screen_info ("%s "%key)
        for statement in p:
          pred = statement["pred"]
          prob = statement["prob"]
          if pred == "same" or pred == "moved":
            op = statement["op"]
            logger.screen_info ("%s %s (%f); "%(pred, op, prob))
          else:
            logger.screen_info ("%s (%f); "%(pred, prob))
        logger.screen_info ("\n")
      if key[0] == "O":
        p = self.results[key]
        p.sort (key=lambda x: x["prob"], reverse=True)
        if p[0]["pred"] == "gone":
          logger.info ("reconstructed: -%s\n"%key)
        logger.screen_info ("%s "%key)
        for statement in p:
          pred = statement["pred"]
          prob = statement["prob"]
          if pred == "same" or pred == "moved":
            op = statement["op"]
            logger.screen_info ("%s %s (%f); "%(pred, op, prob))
          else:
            logger.screen_info ("%s (%f); "%(pred, prob))
        logger.screen_info ("\n")

o = Oracle()
s = Salvatore ()
for i in range(number_steps):
  logger.screen_info ("------------------------------------------------------------------------------------\n")
  logger.screen_info ("Step %i "%i)
  t = time()
  s.do_time_step ()
  t = time () - t
  logger.screen_info ("%f seconds\n"%t)
  logger.info ("time elapsed: %f\n"%t
       + "shelf %s\n"%str(s.shelf)
       + "tables %s\n"%str(s.tables)
       + "newClusters %s\n"%str(s.newClusters)
       + "oldClusters %s\n"%str(s.oldClusters)
       + "evidence %s\n"%str(s.evidence)
       + "----\n")
  if s.results:
    o.read(s.results).interpret()
    logger.info ("results file:\n")
    for line in s.results:
      logger.info (line)
  else:
    logger.info ("no results\n")
