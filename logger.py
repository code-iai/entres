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

