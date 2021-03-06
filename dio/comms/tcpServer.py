import socket
from core.util import debug


class TcpServer(socket.socket):
    def __init__(self, ip_addr, port):
        print("Initializing tcp port on ", (ip_addr, port))
        super().__init__(socket.AF_INET, socket.SOCK_STREAM)
        super().setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        super().bind((ip_addr, port))
        super().setblocking(False)
        super().listen(10)  # Handles up to 10 dropped connections
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
            else:  # Add new client if we have not connected
                print("Client connected ", addr)
                self.clients.append((client, addr))
        except Exception:
            pass

    def recv(self, size=1024):
        for client, addr in self.clients:
            try:
                data = client.recv(size)
                print(data)
                if data != ''.encode('utf-8'):
                    return (addr, data)
                else:
                    client.close()
                    self.clients.remove((client, addr))
                    print("Disconnected from", addr)
            except ConnectionResetError:
                client.close()
                self.clients.remove((client, addr))
                print("Disconnected from", addr)
            except BlockingIOError:
                pass
            except Exception as e:
                debug.print_exception(e)

        return None

    def send(self, addr, data):
        if isinstance(data, str):
            data = data.encode('utf-8')
        for client, caddr in self.clients:
            if addr == caddr or addr is None:
                try:
                    client.send(data)
                except Exception as e:
                    debug.print_exception(e)

    def send_all(self, data):
        self.send(None, data)

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
