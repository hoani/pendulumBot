from RoBus import packet
from RoBus._test.fake import fakeCodec


class TestPacketInstantiation():

  def setup_method(self):
    pass

  def test_create_packet(self):
    p = packet.Packet("get", "protocol", ())
    assert(isinstance(p, packet.Packet))

  def test_contents(self):
    p = packet.Packet("set", "protocol", tuple([1,2,3]))
    assert(p.category == "set")
    assert(p.path == "protocol")
    assert(p.payload[0] == 1)
    assert(p.payload[1] == 2)
    assert(p.payload[2] == 3)
    assert(len(p.payload) == 3)


class TestPacketUnpack():
  def setup_method(self):
    self.codec = fakeCodec.Codec("")

  def test_unpack(self):
    expected = {"protocol/version/major": {"value": 5, "set": False}}
    self.codec.unpack_return = expected
    _packet = packet.Packet("pub")
    result = _packet.unpack(self.codec)
    assert(self.codec.unpack_called == True)
    assert(result == expected)

  

