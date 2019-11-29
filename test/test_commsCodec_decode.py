from source.utilities import commsCodec
import json

class TestGetPacketDecode():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_simple_decoding(self):
    expected = commsCodec.Packet("get", "protocol")
    result = self.codec.decode("G0000\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
  
  def test_nested_decoding(self):
    expected = commsCodec.Packet("get", "protocol/version/patch")
    result = self.codec.decode("G0004\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)

  def test_deep_nest_decoding(self):
    expected = commsCodec.Packet("get", "control/pid/setpoint/value")
    result = self.codec.decode("G800e\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)

  def test_ack_category(self):
    expected = commsCodec.Packet("ack")
    result = self.codec.decode("A\n".encode('utf-8'))
    assert(result.category == expected.category)
  
  def test_nack_category(self):
    expected = commsCodec.Packet("nak")
    result = self.codec.decode("N\n".encode('utf-8'))
    assert(result.category == expected.category)

  def test_simple_packet_decoding(self):
    expected = commsCodec.Packet("get", "protocol/version/major", tuple([0x11]))
    result = self.codec.decode("G0002:11\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload[0] == expected.payload[0])

  def test_multi_packet_decoding(self):
    expected = commsCodec.Packet("get", "protocol/version", tuple([0x11, 0x22, 0x3344]))
    result = self.codec.decode("G0001:11:22:3344\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

class TestSetPacketDecodeMultiple():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_sequential(self):
    expected = commsCodec.Packet("set", "protocol/version", tuple([0x12, 0x34, 0x567]))
    result = self.codec.decode(("S0001:12:34:0567\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_non_sequential(self):
    expected = commsCodec.Packet("set", "protocol", tuple([0x12, 0x34, 0x567, "Hoani"]))
    result = self.codec.decode(("S0000:12:34:0567:Hoani\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_ignores_unused_payload_items(self):
    expected = commsCodec.Packet("set", "protocol/version", tuple([0x12, 0x34, 0x567]))
    result = self.codec.decode(("S0001:12:34:0567\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_ignores_unused_sequential_items(self):
    expected = commsCodec.Packet("set", "protocol/version", tuple([0x12, 0x34]))
    result = self.codec.decode(("S0001:12:34\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)


class TestSetPacketDecodeSingle():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_simple_string(self):
    expected = commsCodec.Packet("set", "typecheck/string", tuple(["Hoani's String"]))
    result = self.codec.decode(("S2001:Hoani's String\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_bool(self):
    expected = commsCodec.Packet("set", "typecheck/boolean", tuple([True]))
    result = self.codec.decode(("S2002:1\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_u8(self):
    expected = commsCodec.Packet("set", "typecheck/uint8", tuple([0xa5]))
    result = self.codec.decode(("S2003:a5\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)
  
  def test_underflow_u8(self):
    expected = commsCodec.Packet("set", "typecheck/uint8", tuple([0x00]))
    result = self.codec.decode(("S2003:-e3\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_overflow_u8(self):
    expected = commsCodec.Packet("set", "typecheck/uint8", tuple([0xff]))
    result = self.codec.decode(("S2003:1a5\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_u16(self):
    expected = commsCodec.Packet("set", "typecheck/uint16", tuple([0x0234]))
    result = self.codec.decode(("S2004:0234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_underflow_u16(self):
    expected = commsCodec.Packet("set", "typecheck/uint16", tuple([0x0000]))
    result = self.codec.decode(("S2004:-0234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_overflow_u16(self):
    expected = commsCodec.Packet("set", "typecheck/uint16", tuple([0xffff]))
    result = self.codec.decode(("S2004:1ffff\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_u32(self):
    expected = commsCodec.Packet("set", "typecheck/uint32", tuple([0x102234]))
    result = self.codec.decode(("S2005:00102234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_underflow_u32(self):
    expected = commsCodec.Packet("set", "typecheck/uint32", tuple([0x00000000]))
    result = self.codec.decode(("S2005:-00102234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_overflow_u32(self):
    expected = commsCodec.Packet("set", "typecheck/uint32", tuple([0xffffffff]))
    result = self.codec.decode(("S2005:100002234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_u64(self):
    expected = commsCodec.Packet("set", "typecheck/uint64", tuple([0x10223400000078]))
    result = self.codec.decode(("S2006:0010223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_underflow_u64(self):
    expected = commsCodec.Packet("set", "typecheck/uint64", tuple([0x0]))
    result = self.codec.decode(("S2006:-10223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_overflow_u64(self):
    expected = commsCodec.Packet("set", "typecheck/uint64", tuple([0xffffffffffffffff]))
    result = self.codec.decode(("S2006:10000010223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_i8(self):
    expected = commsCodec.Packet("set", "typecheck/int8", tuple([0x11]))
    result = self.codec.decode(("S2007:11\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_negative_i8(self):
    expected = commsCodec.Packet("set", "typecheck/int8", tuple([-0x11]))
    result = self.codec.decode(("S2007:ef\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_i16(self):
    expected = commsCodec.Packet("set", "typecheck/int16", tuple([0x0234]))
    result = self.codec.decode(("S2008:0234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_signed_i16(self):
    expected = commsCodec.Packet("set", "typecheck/int16", tuple([-0x0234]))
    result = self.codec.decode(("S2008:fdcc\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_i32(self):
    expected = commsCodec.Packet("set", "typecheck/int32", tuple([0x102234]))
    result = self.codec.decode(("S2009:00102234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_signed_i32(self):
    expected = commsCodec.Packet("set", "typecheck/int32", tuple([-0x102234]))
    result = self.codec.decode(("S2009:ffefddcc\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_simple_i64(self):
    expected = commsCodec.Packet("set", "typecheck/int64", tuple([0x10223400000078]))
    result = self.codec.decode(("S200a:0010223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_signed_i64(self):
    expected = commsCodec.Packet("set", "typecheck/int64", tuple([-0x10223400000078]))
    result = self.codec.decode(("S200a:ffefddcbffffff88\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(result.payload == expected.payload)

  def test_float(self):
    expected = commsCodec.Packet("set", "typecheck/float", tuple([1.2717441261e+20]))
    result = self.codec.decode(("S200b:60dc9cc9\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(abs(result.payload[0] - expected.payload[0]) < 0.00001e+20)

  def test_double(self):
    expected = commsCodec.Packet("set", "typecheck/double", tuple([1.2344999999999999307]))
    result = self.codec.decode(("S200c:3ff3c083126e978d\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.path == expected.path)
    assert(abs(result.payload[0] - expected.payload[0]) < 0.0000001 )
