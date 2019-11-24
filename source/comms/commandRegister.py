class CommandRegister:
  def __init__(self):
    self.registry = dict()
    
  def add(self, command, callback):
    self.registry[command] = callback
    
  def execute(self, command, payload):
    print("have command ", command)
    if command in self.registry.keys():
      self.registry[command](payload)
      return True
    else:
      return False