from pendulumBot.comms import btServer, tcpServer

import time, sys, os
from pendulumBot.bot.robotControl import *
from pendulumBot.bot import motorPair
from pendulumBot.comms import commandRegister
from pendulumBot.utilities import vect, imuData, cli

from external.RoBus.RoBus import codec, packet

import json

import datetime


def print_exception(e):
  exc_type, exc_obj, exc_tb = sys.exc_info()
  fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
  print(exc_type,',', fname,', ln', exc_tb.tb_lineno)
  print(e)
  print("Closing socket")

def get_settings(settings_file_path):
  settings_file_path = 'config/settings.json'
  with open(settings_file_path, "r") as settings_file:
    settings = json.load(settings_file)
  return settings

def check_exit_conditions_rcpy():
  return rcpy.get_state() != rcpy.EXITING

def check_exit_conditions_simulated():
  return True

class RobotControlCallbacks:
  def __init__(self, control):
    self.control = control

  def callback_auto(payload):
    self.control.set_state(RobotControl.STATE_AUTO, payload)
    return True

  def callback_disable(payload):
    self.control.set_state(RobotControl.STATE_DISABLED, None)
    return True

  def callback_manual(payload):
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
      return False

    try:
      speed = float(payload[1])
    except:
      speed = 0.5

    duration_ms = 500
    if len(payload) >= 2:
      try:
        duration_ms = int(payload[2]*1000)
      except:
        pass

    self.control.set_state(RobotControl.STATE_MANUAL, [direction, speed, duration_ms])
    return True


def setup(settings):
  args = cli.get_args(settings["default"])
  simulate = args.simulate

  my_codec = codec.Codec('config/protocol.json')

  hostBtMACAddress = args.bluetooth[0]
  hostBtPort = args.bluetooth[1]

  hostTcpIpAddress = args.tcp[0]
  hostTcpPortCommand = int(args.tcp[1])
  hostTcpPortLogging = int(args.tcp[2])

  if (simulate):
    from pendulumBot.driver.simulated import motors, imu
    check_exit_conditions = check_exit_conditions_simulated

  else:
    import rcpy
    from pendulumBot.driver.rcpy import motors, imu
    rcpy.set_state(rcpy.RUNNING)
    check_exit_conditions = check_exit_conditions_rcpy

  sockets = []

  try:

    left = motors.dcMotor(3)
    right = motors.dcMotor(2)

    pair = motorPair.MotorPair(left, right)

    if (simulate == False):
      sockets.append(btServer.btServer(hostBtMACAddress, hostBtPort))
    sockets.append(tcpServer.TcpServer(hostTcpIpAddress, hostTcpPortCommand))

    robo = RobotControl(pair)
    imu = imu.Imu()

    control_cbs = RobotControlCallbacks(robo)

    commands = commandRegister.CommandRegister()
    commands.add("control/automatic", control_cbs.callback_auto)
    commands.add("control/disable",   control_cbs.callback_disable)
    commands.add("control/manual",    control_cbs.callback_manual)

    last_data = "".encode("utf-8")


  except Exception as e:
    print_exception()
    for sock in sockets:
      sock.close()

  finally:
    pair.stop()

  return (robo, sockets, my_codec, imu, check_exit_conditions)

def run(robo, sockets, my_codec, imu, check_exit_conditions, delta_ms):
  delta_max_ms = 0
  try:
    while check_exit_conditions():
      # TODO: replace sleep timing with interval timing
      last_ms = datetime.datetime.now().timestamp()
      robo.update(delta_ms)

      # comms_update(sockets)
      for sock in sockets:
        sock.accept_connections()
        rx = sock.recv()
        if rx != None:
          # process_packets(rx, sock)
          (addr, data) = rx
          print("\nReceived: {}".format(data))

          (last_data, packets) = my_codec.decode(last_data + data)

          # To do - respond NAK to a group with an unknown command

          for p in packets:
            if p.category == "set":
              response = packet.Packet("ack")
              for (cmd, payload) in tuple(zip(p.paths, p.payloads)):
                response.add(cmd)
                if commands.execute(cmd, payload) == False:
                  print("Command {} Failed", cmd)
                  response.category = "nak"

            sock.send(addr, my_codec.encode(response))

      # publish_subscribed_data(codec, sockets)
      imu_data = imu.sample()

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

      cpu_use_packet = packet.Packet('pub', 'health/os/cpuse', 0.5)
      batt_v_packet = packet.Packet('pub', 'health/batt/v', 12)

      encoded = my_codec.encode(imu_packet) + my_codec.encode(cpu_use_packet) +  my_codec.encode(batt_v_packet)

      for sock in sockets:
        for addr in sock.get_clients():
          sock.send(addr, encoded)

      delta_meas_ms = datetime.datetime.now().timestamp() - last_ms
      delta_max_ms = max(delta_meas_ms, delta_max_ms)
      time.sleep(delta_ms/1000.0)  # sleep some
      print(('\r'
            'overhead (s) = {:0.3f} | max overhead (s) = {:0.3f}'
            ).format(delta_meas_ms, delta_max_ms), end='')

  except Exception as e:
    print_exception(e)
    for sock in sockets:
      sock.close()

  finally:
    robo.wheels.stop()

    # say bye
    print("\nBye BeagleBone!")

if __name__ == "__main__":
  settings = get_settings('config/settings.json')
  items = setup(settings)
  delta_ms = 100
  run(items[0], items[1], items[2], items[3], items[4], delta_ms)


