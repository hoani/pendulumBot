from pendulumBot.comms import btServer, tcpServer

import time
from pendulumBot.bot.robotControl import *
from pendulumBot.bot import motorPair
from pendulumBot.comms import commandRegister

from external.RoBus.RoBus import codec, packet

if __name__ == "__main__":
  use_rcpy = False
  use_bluetooth = False
  
  my_codec = codec.Codec('json/protocol.json')

  hostBtMACAddress = '38:D2:69:E1:11:CB' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters.
  hostBtPort = 3
  
  hostTcpIpAddress = "192.168.1.9"
  hostTcpPort = 11337
  
  def check_exit_conditions_rcpy():
    return rcpy.get_state() != rcpy.EXITING
    
  def check_exit_conditions_simulated():
    return True
  
  if (use_rcpy):
    import rcpy 
    from pendulumBot.driver.rcpy import motors, imu
    rcpy.set_state(rcpy.RUNNING)
    check_exit_conditions = check_exit_conditions_rcpy
  else:
    from pendulumBot.driver.simulated import motors, imu
    check_exit_conditions = check_exit_conditions_simulated
  
  
  def callback_auto(payload):
    global robo
    robo.set_state(RobotControl.STATE_AUTO, payload)
    return True
    
  def callback_disable(payload):
    global robo
    robo.set_state(RobotControl.STATE_DISABLED, None)
    return True
    
  def callback_manual(payload):
    global robo
    if len(payload) < 1:
      return False
    if payload[0] == "FW":
      direction = RobotControl.MANUAL_DIRECTION_FW
    elif payload[0] == "BW":
      direction = RobotControl.MANUAL_DIRECTION_BW
    elif payload[0] == "LT":
      direction = RobotControl.MANUAL_DIRECTION_LT
    elif payload[0] == "RT":
      direction = RobotControl.MANUAL_DIRECTION_RT
    else:
      direction = RobotControl.MANUAL_DIRECTION_FW
    return True
    
    try:
      speed = float(payload[1])
    except:
      speed = 0.5
    
    duration_ms = 500
    if len(payload) > 2:
      try:
        duration_ms = int(payload[2])
      except:
        pass
    
    robo.set_state(RobotControl.STATE_MANUAL, [direction, speed, duration_ms])
  
  
  try:
    
    left = motors.dcMotor(3)
    right = motors.dcMotor(2)
    
    pair = motorPair.MotorPair(left, right)
    if (use_bluetooth):
      bt_socket = btServer.btServer(hostBtMACAddress, hostBtPort)
    tcp_socket = tcpServer.TcpServer(hostTcpIpAddress, hostTcpPort)
    robo = RobotControl(pair)
    imu = imu.Imu()
    delta_ms = 100
    
    commands = commandRegister.CommandRegister()
    commands.add("control/automatic", callback_auto)
    commands.add("control/disable", callback_disable)
    commands.add("control/manual", callback_manual)

    last_data = "".encode("utf-8")
    
    while check_exit_conditions():
      
      
      if use_bluetooth:
        bt_socket.accept_connections()
      tcp_socket.accept_connections()
      if use_bluetooth:
        rx = bt_socket.recv()
        if rx != None:
          srx = bt_socket
        else:
          rx = tcp_socket.recv()
          if rx != None:
            srx = tcp_socket
      else:
        rx = tcp_socket.recv()
        if rx != None:
          srx = tcp_socket
        
      robo.update(delta_ms)
      
      
      if rx != None:
        (addr, data) = rx
        print("Received: {}".format(data))

        (last_data, packets) = my_codec.decode(last_data + data)

        # To do - respond NAK to a group with an unknown command

        for p in packets:
          if p.category == "set":
            response = packet.Packet("ack")
            for (cmd, payload) in tuple(zip(p.paths, p.payloads)):
              response.add(cmd)
              if commands.execute(cmd, payload) == False:
                print("Command {} Failed", cmd)
                p.category = "nak"

            srx.send(addr, my_codec.encode(p))
            
        # lines = data.decode('utf-8').split('\n');
        # for line in lines:
        #   if line == "":
        #     break
        #   args = line.split(" ");
        #   cmd = args[0];
        #   if (len(args) > 1):
        #     payload = args[1:]
        #   else:
        #     payload = []
          
        #   if commands.execute(cmd, payload):
        #     srx.send(addr, "{} ack".format(cmd))
        #   else:
        #     print("Command not recognized")
        #     srx.send(addr, "{} not recognized".format(data.decode('utf-8')))
          
      
      imu_data = imu.sample()
      # imu_packet = (
      #   "accel {:.2f} {:.2f} {:.2f}\n"
      #   "gyros {:.2f} {:.2f} {:.2f}\n"
      #   "magne {:.2f} {:.2f} {:.2f}\n".format(
      #       imu_data.accelerometer.x,
      #       imu_data.accelerometer.y,
      #       imu_data.accelerometer.z,
      #       imu_data.gyroscope.x,
      #       imu_data.gyroscope.y,
      #       imu_data.gyroscope.z,
      #       imu_data.magnetometer.x,
      #       imu_data.magnetometer.y,
      #       imu_data.magnetometer.z,
      #       )
      #     ).encode('utf-8')

      imu_packet = packet.Packet('pub', 'imu', 
        (
          imu_data.accelerometer.x,
          imu_data.accelerometer.y,
          imu_data.accelerometer.z,
          imu_data.gyroscope.x,
          imu_data.gyroscope.y,
          imu_data.gyroscope.z,
          imu_data.magnetometer.x,
          imu_data.magnetometer.y,
          imu_data.magnetometer.z
        )
      )

      imu_bytes = my_codec.encode(imu_packet)
      
      
      if use_bluetooth:
        for addr in bt_socket.get_clients():
          bt_socket.send(addr, imu_bytes)
        
      for addr in tcp_socket.get_clients():
        tcp_socket.send(addr, imu_bytes)
          
        
              
      time.sleep(delta_ms/1000.0)  # sleep some
      
          
  except Exception as e:
    exc_type, exc_obj, exc_tb = sys.exc_info()
    fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
    print(exc_type,',', fname,', ln', exc_tb.tb_lineno)
    print(e)
    print("Closing socket")
    if use_bluetooth:
      bt_socket.close()
    tcp_socket.close()
    
  finally:
    pair.stop()
  
    # say bye
    print("\nBye BeagleBone!")
    
    
  
