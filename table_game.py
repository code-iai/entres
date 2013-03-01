import numpy
import random
import math
from logger import logger

number_objects = 10
number_tables = 1

pose_noise = 0.01
app_noise = 0.05

pose_sigma = 0.2
app_sigma = 0.5

prob_add = 0.5
prob_remove = 0.1
prob_move = 0.3
prob_nothing = 0.1
max_objects_on_table = 5

number_steps = 50
number_actions = 3

class TableState:
  def __init__(self):
    pass
    #self.old_clusters = ["O1"]
    #self.new_clusters = ["N1"]
    #self.similar_appearance = [1.0]
    #self.similar_pose = [1.0]
    #self.out_of_view = []

  def __repr__(self):
    return "old: %s\n"%self.old_clusters + \
           "new: %s\n"%self.new_clusters + \
           "simA: %s\n"%self.similar_appearance + \
           "simP: %s\n"%self.similar_pose + \
           "oov: %s\n"%self.out_of_view
  

class Object:
  def eval_gauss (self, x, mu=0.0, sigma=1.0):
    y = 1 / ( math.sqrt(2. * math.pi * sigma) )  * math.exp( -((x - mu + 0.0) / sigma)**2 )
    return y

  def __init__ (self, name):
    self.name = name
    self.create_appearance ()
    self.create_pose ()
    self.truth = "."

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
    norm = self.eval_gauss (0, sigma=pose_sigma)
    delta = numpy.linalg.norm(self.observePose() - other.observePose())
    return self.eval_gauss (delta, sigma=pose_sigma) / norm

  def observePose (self):
    return self.pose + (numpy.random.randn(2) * pose_noise)
    return self.pose # TODO: this would mean perfect, noiseless vision

  def observe (self):
    return self.app + (numpy.random.randn(5) * app_noise)
    return self.app # TODO: this would mean perfect, noiseless vision

  def similarApp (self, other):
    norm = self.eval_gauss (0, sigma=app_sigma)
    delta = numpy.linalg.norm(self.observe() - other.observe())
    return self.eval_gauss (delta, sigma=app_sigma) / norm

  def __repr__ (self):
    return self.name + self.truth


class TableGame:
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
    obj.truth = '+'
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
    obj.truth = '-'
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
    obj.truth = '~'
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

  def do_time_step (self):
    self.oldClusters = self.newClusters
    for o in self.oldClusters:
      o.truth = '.'
      for table in self.tables:
        for o in table:
          o.truth = '.'

    self.newClusters = []
    for _ in range (number_actions):
      done = False
      while not done:
        action = self.select_action ()
        if action:
          done = action ()
    table = random.randrange (0, number_tables) 
    self.observe_scene (table)
    logger.screen_info ("(%i x %i) "%(len(self.oldClusters),len(self.newClusters)))

    self.cur_iter += 1

  def observe_scene (self, table):
    self.newClusters = []
    for t in range(len(self.tables)):
      if t == table:
        for obj in self.tables[t]:
          self.newClusters.append (obj)

  def get_table_state (self):
    state = TableState()
    state.old_clusters = self.oldClusters
    state.new_clusters = self.newClusters
    state.similar_appearance = []
    state.similar_pose = []
    for n in self.newClusters:
      for o in self.oldClusters:
        state.similar_appearance.append (o.similarApp(n))
        state.similar_pose.append (o.similarPos(n))

    state.out_of_view = []
    print state
    return state
