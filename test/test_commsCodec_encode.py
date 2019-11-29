from source.utilities import commsCodec
import json

class TestAckPacketEncode():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_ack_encoding(self):
    expected = ("A\n").encode('utf-8')
    packet = commsCodec.Packet("ack")
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_nack_encoding(self):
    expected = ("N\n").encode('utf-8')
    packet = commsCodec.Packet("nak")
    result = self.codec.encode(packet)
    assert(result == expected)


class TestGetPacketEncode():

  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_simple_encoding(self):
    expected = ("G0000\n").encode('utf-8')
    packet = commsCodec.Packet("get", "protocol")
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_nested_encoding(self):
    expected = ("G0004\n").encode('utf-8')
    packet = commsCodec.Packet("get", "protocol/version/patch")
    result = self.codec.encode(packet)
    assert(result == expected)



class TestSetPacketEncodeMultiple():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_sequential(self):
    expected = ("S0001:12:34:0567\n").encode('utf-8')
    packet = commsCodec.Packet("set", "protocol/version", tuple([0x12, 0x34, 0x567]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_non_sequential(self):
    expected = ("S0000:12:34:0567:Hoani\n").encode('utf-8')
    packet = commsCodec.Packet("set", "protocol", tuple([0x12, 0x34, 0x567, "Hoani"]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_ignores_unused_payload_items(self):
    expected = ("S0001:12:34:0567\n").encode('utf-8')
    packet = commsCodec.Packet("set", "protocol/version", tuple([0x12, 0x34, 0x567]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_ignores_unused_sequential_items(self):
    expected = ("S0001:12:34\n").encode('utf-8')
    packet = commsCodec.Packet("set", "protocol/version", tuple([0x12, 0x34]))
    result = self.codec.encode(packet)
    assert(result == expected)


class TestSetPacketEncodeSingle():
  def setup_method(self):
    protocol_file_path = "test/fakes/protocol.json"
    self.codec = commsCodec.Codec(protocol_file_path)

  def test_simple_string(self):
    expected = ("S2001:Hoani's String\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/string", tuple(["Hoani's String"]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_bool(self):
    expected = ("S2002:1\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/boolean", tuple([True]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_u8(self):
    expected = ("S2003:a5\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint8", tuple([0xa5]))
    result = self.codec.encode(packet)
    assert(result == expected)
  
  def test_underflow_u8(self):
    expected = ("S2003:00\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint8", tuple([-0xa5]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_u8(self):
    expected = ("S2003:ff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint8", tuple([0x1a5]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_u16(self):
    expected = ("S2004:0234\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint16", tuple([0x0234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_u16(self):
    expected = ("S2004:0000\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint16", tuple([-0x0234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_u16(self):
    expected = ("S2004:ffff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint16", tuple([0x10234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_u32(self):
    expected = ("S2005:00102234\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint32", tuple([0x102234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_u32(self):
    expected = ("S2005:00000000\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint32", tuple([-0x102234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_u32(self):
    expected = ("S2005:ffffffff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint32", tuple([0x100002234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_u64(self):
    expected = ("S2006:0010223400000078\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint64", tuple([0x10223400000078]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_u64(self):
    expected = ("S2006:0000000000000000\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint64", tuple([-0x10223400000078]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_u64(self):
    expected = ("S2006:ffffffffffffffff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/uint64", tuple([0x10000010223400000078]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_i8(self):
    expected = ("S2007:11\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int8", tuple([0x11]))
    result = self.codec.encode(packet)
    assert(result == expected)  

  def test_negative_i8(self):
    expected = ("S2007:ef\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int8", tuple([-0x11]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_i8(self):
    expected = ("S2007:7f\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int8", tuple([0x1FF]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_i8(self):
    expected = ("S2007:80\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int8", tuple([-0x1FF]))
    result = self.codec.encode(packet)
    assert(result == expected)
  
  def test_simple_i16(self):
    expected = ("S2008:0234\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int16", tuple([0x0234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_signed_i16(self):
    expected = ("S2008:fdcc\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int16", tuple([-0x0234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_i16(self):
    expected = ("S2008:7fff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int16", tuple([0x1ffff]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_i16(self):
    expected = ("S2008:8000\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int16", tuple([-0x1FF00]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_i32(self):
    expected = ("S2009:00102234\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int32", tuple([0x102234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_signed_i32(self):
    expected = ("S2009:ffefddcc\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int32", tuple([-0x102234]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_i32(self):
    expected = ("S2009:7fffffff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int32", tuple([0x1fffffffff]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_i32(self):
    expected = ("S2009:80000000\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int32", tuple([-0x1FF0000000]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_simple_i64(self):
    expected = ("S200a:0010223400000078\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int64", tuple([0x10223400000078]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_signed_i64(self):
    expected = ("S200a:ffefddcbffffff88\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int64", tuple([-0x10223400000078]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_overflow_i64(self):
    expected = ("S200a:7fffffffffffffff\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int64", tuple([0x1ffffffffffffffff0]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_underflow_i64(self):
    expected = ("S200a:8000000000000000\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/int64", tuple([-0x1FF000000000000000]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_float(self):
    expected = ("S200b:60dc9cc9\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/float", tuple([1.2717441261e+20]))
    result = self.codec.encode(packet)
    assert(result == expected)

  def test_double(self):
    expected = ("S200c:3ff3c083126e978d\n").encode('utf-8')
    packet = commsCodec.Packet("set", "typecheck/double", tuple([1.2344999999999999307]))
    result = self.codec.encode(packet)
    assert(result == expected)

