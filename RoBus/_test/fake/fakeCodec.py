class Codec():
  def __init__(self, protocol_file_path):
   self.filepath = protocol_file_path
   self.protocol = {}
   self.encode_called = False
   self.decode_called = False
   self.unpack_called = False
   self.unpack_return = {}

  def encode(self, packets):
    self.encode_called = True
    return "".encode('utf-8')

  def decode(self, encoded):
    self.decode_called = True
    return ("".encode('utf-8'), [])

  def unpack(self, packet):
    self.unpack_called = True
    return self.unpack_return

  def category_from_start(self, start):
    return None

  def path_from_address(self, address):
    return ""

  def struct_from_address(self, address):
    return {}

  def is_settable(self, address):
    return False
    