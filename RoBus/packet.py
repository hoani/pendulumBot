# RoBus Codec
# 2019 (C) Hoani Bryson

class Packet():
  def __init__(self, category, path=None, payload=None):
    self.category = category
    self.paths = []
    self.payloads = []
    self.add(path, payload)

  def add(self, path, payload):
    self.paths.append(path)
    if isinstance(payload, tuple) == False and payload != None:
      self.payloads.append(tuple([payload]))
    else:
      self.payloads.append(payload)

  def unpack(self, codec):
    return codec.unpack(self)


