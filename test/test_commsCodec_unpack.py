from source.utilities import commsCodec
import json

class TestPacketPayloadUnpack():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_unpack_simple(self):
    expected = {"protocol/version/major": {"value": 5, "set": False}}
    packet = commsCodec.Packet("pub", "protocol/version/major", 5)
    result = self.codec.unpack(packet)
    assert(result == expected)

  def test_unpack_multiple(self):
    expected = {
      "protocol/version/major": {"value": 5, "set": False},
      "protocol/version/minor": {"value": 4, "set": False},
      "protocol/version/patch": {"value": 3, "set": False},
    }
    packet = commsCodec.Packet("pub", "protocol/version", (5, 4, 3))
    result = self.codec.unpack(packet)
    assert(result == expected)

  def test_unpack_partial(self):
    expected = {
      "protocol/version/major": {"value": 5, "set": False},
      "protocol/version/minor": {"value": 4, "set": False},
    }
    packet = commsCodec.Packet("pub", "protocol/version", (5, 4))
    result = self.codec.unpack(packet)
    assert(result == expected)

  def test_unpack_overflow(self):
    expected = {
      "protocol/version/major": {"value": 5, "set": False},
      "protocol/version/minor": {"value": 4, "set": False},
      "protocol/version/patch": {"value": 3, "set": False}
    }
    packet = commsCodec.Packet("pub", "protocol/version", (5, 4, 3, 2))
    result = self.codec.unpack(packet)
    assert(result == expected)

  def test_unpack_settable(self):
    expected = {"typecheck/uint32": {"value": 0x12345, "set": True}}
    packet = commsCodec.Packet("pub", "typecheck/uint32", 0x12345)
    result = self.codec.unpack(packet)
    assert(result == expected)

  def test_unpack_explore(self):
    expected = {
      "imu/accel/x": {"value": 12.0, "set": False},
      "imu/accel/y": {"value": 23.0, "set": False},
      "imu/accel/z": {"value": 45.0, "set": False},
      "imu/gyros/x": {"value": 67.0, "set": False},
      "imu/gyros/y": {"value": 89.0, "set": False},
      "imu/gyros/z": {"value": 12.0, "set": False},
      "imu/magne/x": {"value": 34.0, "set": False},
      "imu/magne/y": {"value": 56.0, "set": False},
      "imu/magne/z": {"value": 78.0, "set": False}
    }
    packet = commsCodec.Packet("pub", "imu", tuple([12.0, 23.0, 45.0, 67.0, 89.0, 12.0, 34.0, 56.0, 78.0]))
    result = self.codec.unpack(packet)
    assert(result == expected)

