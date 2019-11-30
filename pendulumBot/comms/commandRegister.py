class CommandRegister:
  def __init__(self):
    self.registry = dict()
    
  def add(self, command, callback):
    self.registry[command] = callback
    
  def execute(self, command, payload):
    print("have command", command, payload)
    if command in self.registry.keys():
      return self.registry[command](payload)
    else:
      return False