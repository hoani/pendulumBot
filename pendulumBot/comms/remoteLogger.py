from pendulumBot.utilities import csvLines

class RemoteLogger:
  EVERY_UPDATE = None
  def __init__(self, sock, period_ms = EVERY_UPDATE):
    self.socket = sock
    self.period_ms = period_ms
    self.current_ms = 0
    self.next_ms = 0
    self.started = False
    self.csv_lines = csvLines.CsvLines(',', '/n')

  def add(self, key):
    self.csv_lines.add(key)

  def set(self, key, value):
    self.csv_lines.set(key, value)

  def update(self, delta_ms):
    if self.started:
      self.current_ms += delta_ms
      if self.period_ms == EVERY_UPDATE:
        post = True
      elif self.next_ms <= self.current_ms:
        post = True
        while self.next_ms <= self.current_ms:
          self.next_ms += self.period_ms
      else:
        post = False

      if post:
        if self.csv_lines.ready():
          line = self.csv_lines.pop_string()
          self.socket.send_all(line)
        else:
          print("Warning: the remote logger is not keeping up")

  def start(self):
    self.started = True
    self.current_ms = 0
    self.next_ms = delta_ms

  def stop(self):
    self.started = False
