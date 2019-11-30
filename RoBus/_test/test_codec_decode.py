from RoBus import codec, packet
import json, os

CONFIG_PATH = os.path.dirname(__file__) + "/fake/protocol.json"

class TestGetPacketDecode():
  def setup_method(self):
    protocol_file_path = CONFIG_PATH
    self.codec = codec.Codec(protocol_file_path)

  def test_simple_decoding(self):
    expected = packet.Packet("get", "protocol")
    (_, [result]) = self.codec.decode("G0000\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)

  def test_remainder(self):
    expected = "S10".encode('utf-8')
    (result, packets) = self.codec.decode("G0000\nS10".encode('utf-8'))
    assert(result == expected)
    assert(len(packets) == 1)
  
  def test_incomplete(self):
    expected = "S10".encode('utf-8')
    (result, packets) = self.codec.decode("S10".encode('utf-8'))
    assert(result == expected)
    assert(len(packets) == 0)
  
  def test_nested_decoding(self):
    expected = packet.Packet("get", "protocol/version/patch")
    (_, [result]) = self.codec.decode("G0004\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)

  def test_deep_nest_decoding(self):
    expected = packet.Packet("get", "control/pid/setpoint/value")
    (_, [result]) = self.codec.decode("G800e\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)

  def test_ack_category(self):
    expected = packet.Packet("ack")
    (_, [result]) = self.codec.decode("A\n".encode('utf-8'))
    assert(result.category == expected.category)
  
  def test_nack_category(self):
    expected = packet.Packet("nak")
    (_, [result]) = self.codec.decode("N\n".encode('utf-8'))
    assert(result.category == expected.category)

  def test_simple_payload_decoding(self):
    expected = packet.Packet("get", "protocol/version/major", tuple([0x11]))
    (_, [result]) = self.codec.decode("G0002:11\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_multi_payload_decoding(self):
    expected = packet.Packet("get", "protocol/version", tuple([0x11, 0x22, 0x3344]))
    (_, [result]) = self.codec.decode("G0001:11:22:3344\n".encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

class TestDecodeCompoundPackets():
  def setup_method(self):
    protocol_file_path = CONFIG_PATH
    self.codec = codec.Codec(protocol_file_path)

  def test_compound_decoding(self):
    expected = packet.Packet("get", "protocol")
    expected.add("protocol/version")
    (_, result) = self.codec.decode("G0000|0001\n".encode('utf-8'))
    assert(len(result) == 1)
    assert(result[0].category == expected.category)
    assert(result[0].paths == expected.paths)

  def test_compound_with_payloads_decoding(self):
    expected = packet.Packet("set", "protocol", (0x11, 0x22, 0x33))
    expected.add("protocol/version", 0x44)
    (_, result) = self.codec.decode("S0000:11:22:33|0001:44\n".encode('utf-8'))
    assert(len(result) == 1)
    assert(result[0].category == expected.category)
    assert(result[0].paths == expected.paths)
    assert(result[0].payloads == expected.payloads)

  def test_multipacket_compound_decoding(self):
    expected = [
      packet.Packet("get", "protocol"),
      packet.Packet("sub", "control")
    ]
    expected[0].add("protocol/version")
    expected[1].add("imu/accel")
    (_, result) = self.codec.decode("G0000|0001\nB8000|1201\n".encode('utf-8'))
    assert(len(result) == 2)
    assert(result[0].category == expected[0].category)
    assert(result[0].paths    == expected[0].paths)
    assert(result[1].category == expected[1].category)
    assert(result[1].paths    == expected[1].paths)

class TestSetPayloadDecodeMultiple():
  def setup_method(self):
    protocol_file_path = CONFIG_PATH
    self.codec = codec.Codec(protocol_file_path)

  def test_sequential(self):
    expected = packet.Packet("set", "protocol/version", tuple([0x12, 0x34, 0x567]))
    (_, [result]) = self.codec.decode(("S0001:12:34:0567\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_non_sequential(self):
    expected = packet.Packet("set", "protocol", tuple([0x12, 0x34, 0x567, "Hoani"]))
    (_, [result]) = self.codec.decode(("S0000:12:34:0567:Hoani\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_ignores_unused_payload_items(self):
    expected = packet.Packet("set", "protocol/version", tuple([0x12, 0x34, 0x567]))
    (_, [result]) = self.codec.decode(("S0001:12:34:0567\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_ignores_unused_sequential_items(self):
    expected = packet.Packet("set", "protocol/version", tuple([0x12, 0x34]))
    (_, [result]) = self.codec.decode(("S0001:12:34\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)


class TestSetPayloadDecodeSingle():
  def setup_method(self):
    protocol_file_path = CONFIG_PATH
    self.codec = codec.Codec(protocol_file_path)

  def test_simple_string(self):
    expected = packet.Packet("set", "typecheck/string", tuple(["Hoani's String"]))
    (_, [result]) = self.codec.decode(("S2001:Hoani's String\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_bool(self):
    expected = packet.Packet("set", "typecheck/boolean", tuple([True]))
    (_, [result]) = self.codec.decode(("S2002:1\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_u8(self):
    expected = packet.Packet("set", "typecheck/uint8", tuple([0xa5]))
    (_, [result]) = self.codec.decode(("S2003:a5\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)
  
  def test_underflow_u8(self):
    expected = packet.Packet("set", "typecheck/uint8", tuple([0x00]))
    (_, [result]) = self.codec.decode(("S2003:-e3\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_overflow_u8(self):
    expected = packet.Packet("set", "typecheck/uint8", tuple([0xff]))
    (_, [result]) = self.codec.decode(("S2003:1a5\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_u16(self):
    expected = packet.Packet("set", "typecheck/uint16", tuple([0x0234]))
    (_, [result]) = self.codec.decode(("S2004:0234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_underflow_u16(self):
    expected = packet.Packet("set", "typecheck/uint16", tuple([0x0000]))
    (_, [result]) = self.codec.decode(("S2004:-0234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_overflow_u16(self):
    expected = packet.Packet("set", "typecheck/uint16", tuple([0xffff]))
    (_, [result]) = self.codec.decode(("S2004:1ffff\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_u32(self):
    expected = packet.Packet("set", "typecheck/uint32", tuple([0x102234]))
    (_, [result]) = self.codec.decode(("S2005:00102234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_underflow_u32(self):
    expected = packet.Packet("set", "typecheck/uint32", tuple([0x00000000]))
    (_, [result]) = self.codec.decode(("S2005:-00102234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_overflow_u32(self):
    expected = packet.Packet("set", "typecheck/uint32", tuple([0xffffffff]))
    (_, [result]) = self.codec.decode(("S2005:100002234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_u64(self):
    expected = packet.Packet("set", "typecheck/uint64", tuple([0x10223400000078]))
    (_, [result]) = self.codec.decode(("S2006:0010223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_underflow_u64(self):
    expected = packet.Packet("set", "typecheck/uint64", tuple([0x0]))
    (_, [result]) = self.codec.decode(("S2006:-10223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_overflow_u64(self):
    expected = packet.Packet("set", "typecheck/uint64", tuple([0xffffffffffffffff]))
    (_, [result]) = self.codec.decode(("S2006:10000010223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_i8(self):
    expected = packet.Packet("set", "typecheck/int8", tuple([0x11]))
    (_, [result]) = self.codec.decode(("S2007:11\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_negative_i8(self):
    expected = packet.Packet("set", "typecheck/int8", tuple([-0x11]))
    (_, [result]) = self.codec.decode(("S2007:ef\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_i16(self):
    expected = packet.Packet("set", "typecheck/int16", tuple([0x0234]))
    (_, [result]) = self.codec.decode(("S2008:0234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_signed_i16(self):
    expected = packet.Packet("set", "typecheck/int16", tuple([-0x0234]))
    (_, [result]) = self.codec.decode(("S2008:fdcc\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_i32(self):
    expected = packet.Packet("set", "typecheck/int32", tuple([0x102234]))
    (_, [result]) = self.codec.decode(("S2009:00102234\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_signed_i32(self):
    expected = packet.Packet("set", "typecheck/int32", tuple([-0x102234]))
    (_, [result]) = self.codec.decode(("S2009:ffefddcc\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_simple_i64(self):
    expected = packet.Packet("set", "typecheck/int64", tuple([0x10223400000078]))
    (_, [result]) = self.codec.decode(("S200a:0010223400000078\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_signed_i64(self):
    expected = packet.Packet("set", "typecheck/int64", tuple([-0x10223400000078]))
    (_, [result]) = self.codec.decode(("S200a:ffefddcbffffff88\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(result.payloads == expected.payloads)

  def test_float(self):
    expected = packet.Packet("set", "typecheck/float", tuple([1.2717441261e+20]))
    (_, [result]) = self.codec.decode(("S200b:60dc9cc9\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(abs(result.payloads[0][0] - expected.payloads[0][0]) < 0.00001e+20)

  def test_double(self):
    expected = packet.Packet("set", "typecheck/double", tuple([1.2344999999999999307]))
    (_, [result]) = self.codec.decode(("S200c:3ff3c083126e978d\n").encode('utf-8'))
    assert(result.category == expected.category)
    assert(result.paths == expected.paths)
    assert(abs(result.payloads[0][0] - expected.payloads[0][0]) < 0.0000001 )
