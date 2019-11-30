# RoBus Codec
# 2019 (C) Hoani Bryson

from RoBus import packet

import json, struct

def count_depth(root):
  count = 0
  if "type" in root.keys():
    return 0

  for key in root.keys():
    if key[0] != "_":
      count += 1
      count += count_depth(root[key])

  return count


def count_to_path(root, path):
  count = 0
  if path == None:
    return count_depth(root)
  
  search = path[0]
  if search in root.keys():
    for key in root.keys():
      if key[0] != "_":
        count += 1
        if key != search:
          if "type" in root[key]:
            pass
          else:
            count += count_depth(root[key])
        else:
          break
    
    if len(path) > 1:
      incr = count_to_path(root[search], path[1:])
      if (incr != None):
        count += incr
      else:
        return None

  else:
    return None

  return count

def path_from_count(root, count):
  path = [""]
  if count <= 0:
    return ([], 0)
  else:
    for key in root.keys():
      if key[0] != "_":
        count -= 1
        path[0] = key
        if "type" not in root[key]:
          (npath, count) = path_from_count(root[key], count)
          path = path + npath

        if count == 0:
          break
    else:
      # did not count to 0, return an empty path
      path = []
  return (path, count)

def get_struct(root, path):
  if path == []:
    return root

  if path[0] in root.keys():
    if len(path) == 1:
      return root[path[0]]
    else:
      return get_struct(root[path[0]], path[1:])
  else:
    return None

def extract_types(root, path):
  start = get_struct(root, path)
  types = []
  if start != None:
    if "type" in start.keys():
      types.append(start["type"])
    else:
      for key in start.keys():
        if key[0] != "_":
          types = types + extract_types(start[key], [])

  return types

def extract_path_settable_pairs(root, path):
  start = get_struct(root, path)
  paths = []
  settables = []
  if start != None:
    if "type" in start.keys():
      if "set" in start.keys():
        return ([""], [start["set"]])
      else:
        return ([], [])
    else:
      for key in start.keys():
        if key[0] != "_":
          (deep_paths, deep_settables) = extract_path_settable_pairs(start[key], [])
          if deep_paths == [""]:
            paths.append(key)
            settables += deep_settables
          else:
            for (deep_path, deep_settable) in tuple(zip(deep_paths, deep_settables)):
              paths.append("/".join([key, deep_path]))
              settables.append(deep_settable)

  return (paths, settables)

def clamp(value, min_value, max_value):
  return max(min_value, min(value, max_value))
  
def encode_types(item, typeof):
  if typeof == "u8":
    return "{:02x}".format(clamp(item, 0x00, 0xff))
  elif typeof == "u16":
    return "{:04x}".format(clamp(item, 0x0000, 0xffff))
  elif typeof == "u32":
    return "{:08x}".format(clamp(item, 0x00000000, 0xffffffff))
  elif typeof == "u64":
    return "{:016x}".format(clamp(item, 0x0000000000000000, 0xffffffffffffffff))
  if typeof == "i8":
    item = clamp(item, -0x80, 0x7F)
    return "{:02x}".format(item + 0x100 if item < 0 else item)
  elif typeof == "i16":
    item = clamp(item, -0x8000, 0x7FFF)
    return "{:04x}".format(item + 0x10000 if item < 0 else item)
  elif typeof == "i32":
    item = clamp(item, -0x80000000, 0x7FFFFFFF)
    return "{:08x}".format(item + 0x100000000 if item < 0 else item)
  elif typeof == "i64":
    item = clamp(item, -0x8000000000000000, 0x7FFFFFFFFFFFFFFF)
    return "{:016x}".format(item + 0x10000000000000000 if item < 0 else item)
  elif typeof == "string":
    return item
  elif typeof == "bool":
    return "1" if item == True else "0"
  elif typeof == "float":
    return ''.join(format(x, '02x') for x in struct.pack('>f', item))
  elif typeof == "double":
    return ''.join(format(x, '02x') for x in struct.pack('>d', item))
  else:
    return ""

def decode_unsigned(item, bits):
  try:
    return clamp(int(item, 16), 0, (0x1 << bits) - 1)
  except:
    return 0

def decode_signed(item, bits):
  try:
    value = int(item, 16)
    min_value = 0x1 << (bits - 1)
    if value > min_value:
      value -= 0x1 << (bits)

    return clamp(value, -min_value, min_value -1)
  except:
    return 0

def decode_types(item, typeof):
  if typeof == "u8":
    return decode_unsigned(item, 8)
  elif typeof == "u16":
    return decode_unsigned(item, 16)
  elif typeof == "u32":
    return decode_unsigned(item, 32)
  elif typeof == "u64":
    return decode_unsigned(item, 64)
  if typeof == "i8":
    return decode_signed(item, 8)
  elif typeof == "i16":
    return decode_signed(item, 16)
  elif typeof == "i32":
    return decode_signed(item, 32)
  elif typeof == "i64":
    return decode_signed(item,64)
  elif typeof == "string":
    return item
  elif typeof == "bool":
    return True if item == "1" else False
  elif typeof == "float":
    [x] = struct.unpack('>f', bytearray.fromhex(item))
    return x
  elif typeof == "double":
    [x] = struct.unpack('>d', bytearray.fromhex(item))
    return x
  else:
    return ""

class Codec():
  def __init__(self, protocol_file_path):
    with open(protocol_file_path, "r") as protocol_file:
      self.protocol = json.load(protocol_file)
    self._generate_address_map()
    self._generate_category_map()

  def encode(self, packets):
    if isinstance(packets, packet.Packet):
      packets = [packets]
    elif not isinstance(packets, list):
      return ""

    encoded = ""
    for _packet in packets:
      if encoded != "":
        encoded += self.protocol["end"]

      encoded += self.protocol["category"][_packet.category]

      for (ppath, ppayload) in tuple(zip(_packet.paths, _packet.payloads)):
        if ppath != None:
          path = ppath.split("/")
          root = self.protocol["data"][path[0]]
          address = int(root["_addr"], 16)
          if len(path) > 1:
            incr = count_to_path(root, path[1:])
            if (incr != None):
              address += incr
            else:
              print("invalid address: {}".format(ppath))
              return "".encode("utf-8")
          encoded += "{:04x}".format(address)
        if ppayload != None:
          types = extract_types(root, path[1:])
          count = min(len(types), len(ppayload))
          for i in range(count):
            encoded += self.protocol["separator"]
            encoded += encode_types(ppayload[i], types[i])

    encoded += self.protocol["end"]
    return encoded.encode('utf-8')


  ## Decodes an incoming packet stream
  # Inputs: <byte-string> encoded 
  # Returns: Tuple(<byte-string> remainder, Array[<packet>] Packets)
  #
  def decode(self, encoded):
    strings = encoded.split(self.protocol["end"].encode('utf-8'))
    remainder = strings[-1]
    packets = []
    if len(strings) == 1:
      return (remainder, [])

    strings = strings[0:-1]
    for string in strings:
      print(string)
      string = string.decode('utf-8')
      category = None
      path = None
      start = string[0]
      category = self.category_from_start(start)
      _packet = packet.Packet(category)
      subpackets = string[1:].split(self.protocol["compound"])
      print(subpackets)
      for subpacket in subpackets:
        parts = subpacket.split(self.protocol["separator"])
        if parts != ['']:
          payload = []
          addr = parts[0]
          path = self.path_from_address(addr)
          path_array = path.split("/")
          print(path)
          root = self.protocol["data"][path_array[0]]
          types = extract_types(root, path_array[1:])
          for (item, typeof) in tuple(zip(parts[1:], types)):
            payload.append(decode_types(item, typeof))

          payload = tuple(payload)

          _packet.add(path, payload)

      packets.append(_packet)
    
    return (remainder, packets)

  def unpack(self, _packet):
    result = {}
    for ppath, ppayload in tuple(zip(_packet.paths, _packet.payloads)) :
      (paths, settables) = extract_path_settable_pairs(self.protocol["data"], ppath.split("/"))
      if paths == [""] and len(ppayload) == 1:
        result[ppath] = {"value": ppayload[0], "set": settables[0]}
      else: 
        for (path_end, settable, value) in tuple(zip(paths, settables, ppayload)):
          path = "/".join([ppath] + [path_end])
          result[path] = {"value": value, "set": settable}
    return result

  def category_from_start(self, start):
    if start in self.category_map.keys():
      return self.category_map[start]
    else:
      return None

  def path_from_address(self, address):
    try:
      int(address, 16)
    except:
      return ""


    keys = self.address_map.keys()
    for i, key in enumerate(keys):
      if i + 1 == len(keys):
        next_key = str(max(int(address, 16), int(key, 16)))
      else:
        next_key = list(keys)[i + 1]

      if clamp(int(address, 16), int(key, 16), int(next_key, 16)-1) == int(address, 16):
        diff = int(address, 16) - int(key, 16)
        (path, count) = path_from_count(
          self.protocol["data"][self.address_map[key]], 
          diff
        )
        if (count == 0):
          return "/".join([self.address_map[key]] + path)
        else:
          return ""

    return ""

  def struct_from_address(self, address):
    path = self.path_from_address(address)
    struct = self.protocol["data"]
    return struct[path]

  def is_settable(self, address):
    struct = get_struct(self.protocol["data"], address.split("/"))
    if struct == None:
      # Cannot set something not part of the protocol
      return False
    else:
      if "set" in struct:
        return struct["set"]
      else:
        # An incomplete address may or may not be settable
        return None

  def _generate_address_map(self):
    self.address_map = {}
    for key in self.protocol["data"].keys():
      if key[0] != "_":
        if "_addr" in self.protocol["data"][key]:
          self.address_map[self.protocol["data"][key]["_addr"]] = key
    # Sort the map
    self.address_map = dict(sorted(self.address_map.items()))

  def _generate_category_map(self):
    self.category_map = {}
    for key in self.protocol["category"].keys():
      self.category_map[self.protocol["category"][key]] = key
    

