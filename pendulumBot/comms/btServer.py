try:
  import bluetooth

  class btServer(bluetooth.BluetoothSocket):
    def __init__(self, mac_addr, port):
      print("Initializing bluetooth on ", mac_addr, ":{:05}".format(port))
      super().__init__(bluetooth.RFCOMM)
      super().bind((mac_addr, port))
      super().setblocking(False)
      super().listen(10) # Handles up to 10 dropped connections
      self.clients = []

    def accept_connections(self):
      try:
        client, addr = super().accept()
        client.setblocking(False)

        # Update client info if we have already connected before
        for idx, (stored_client, stored_addr) in enumerate(self.clients):
          if (stored_addr == addr):
            print("Client reconnected ", addr)
            self.clients[idx] = (client, addr)
            break
        else: # Add new client if we have not connected
          print("Client connected ", addr)
          self.clients.append((client, addr))
      except:
        pass

    def recv(self, size=1024):
      for client, addr in self.clients:
        try:
          data = client.recv(size)
          if data != False:
            return (addr, data)
        except:
          pass
      else:
        return None

    def send(self, addr, data):
      for client, caddr in self.clients:
        if addr == caddr:
          try:
            client.send(data)
          except:
            pass

    def get_clients(self):
      addresses = []
      for client, addr in self.clients:
        addresses.append(addr)

      return addresses

    def close(self):
      for client, addr in self.clients:
        client.close()
      super().close()
      self.clients = []


  if __name__ == "__main__":
    hostMACAddress = '38:D2:69:E1:11:CB' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters.
    port = 3
    backlog = 1
    size = 1024
    s = btServer(hostMACAddress, port)
    try:
      while(len(s.get_clients()) == 0):
        s.accept_connections();

      while 1:
        s.accept_connections();
        rx = s.recv()
        if rx != None:
          (addr, data) = rx
          print(data)
          s.send(addr, data) # Echo back to client

    except Exception as e:
        print(e)
        print("Closing socket")
        s.close()
except:
  print("bluetooth unavaliable")