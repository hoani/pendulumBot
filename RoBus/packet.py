# RoBus Codec
# 2019 (C) Hoani Bryson

class Packet():
  def __init__(self, category, path=None, payload=None):
    self.category = category
    self.path = path
    if isinstance(payload, tuple) == False and payload != None:
      self.payload = tuple([payload])
    else:
      self.payload = payload

  def unpack(self, codec):
    return codec.unpack(self)


