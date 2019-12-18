
class CsvLines(dict):
  TERMINATOR = "/n"
  SEPARATOR = ","
  def __init__(self, separator=SEPARATOR, terminator=TERMINATOR):
    super().__init__()
    self.separator = separator
    self.terminator = terminator

  def add(self, key):
    self[key] = None

  def set(self, key, value):
    if isinstance(key, list):
      N = min(len(key), len(value))
      for i in range(N):
        self.set(key[i], value[i])
    else:
      if key in self.keys():
        self[key] = value

  def ready(self):
    if None in self.values():
      return False
    else:
      return True

  def pop_string(self):
    if self.ready() == False:
      return ""

    line = self.separator.join(self.values())

    for k in self.keys():
      self[k] = None

    return line + self.terminator

  def header_string(self):
    headings = self.separator.join(self.keys())
    return headings + self.terminator