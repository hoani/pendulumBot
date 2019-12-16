from pendulumBot.comms import btServer, tcpServer

import time, sys, os
from pendulumBot.bot.robotControl import *
from pendulumBot.bot import motorPair, ahrs
from pendulumBot.comms import commandRegister, commandCallbacks
from pendulumBot.utilities import vect, imuData, cli, debug

from external.RoBus.RoBus import codec, packet

import json

import datetime




def get_settings(settings_file_path):
  settings_file_path = 'config/settings.json'
  with open(settings_file_path, "r") as settings_file:
    settings = json.load(settings_file)
  return settings

def check_exit_conditions_rcpy():
  return rcpy.get_state() != rcpy.EXITING

def check_exit_conditions_simulated():
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
    my_ahrs = ahrs.AhrsTwoWheeled() 

    rc_callbacks = commandCallbacks.RobotControlCallbacks(robo)

    registry = commandRegister.CommandRegister()
    rc_callbacks.register(registry)

    last_data = "".encode("utf-8")


  except Exception as e:
    debug.print_exception(e)
    for sock in sockets:
      sock.close()

  finally:
    pair.stop()

  return (robo, sockets, my_codec, imu, my_ahrs, check_exit_conditions)

def run(robo, sockets, my_codec, imu, my_ahrs, check_exit_conditions, delta_ms):
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

      ahrs_imu_data = imuData.ImuData(
        vect.Vec3(
          -imu_data.accelerometer.z,
          -imu_data.accelerometer.x,
          imu_data.accelerometer.y
        ),
        vect.Vec3(
          -imu_data.gyroscope.z,
          -imu_data.gyroscope.x,
          imu_data.gyroscope.y
        ),
        vect.Vec3(
          -imu_data.magnetometer.z,
          -imu_data.magnetometer.x,
          imu_data.magnetometer.y
        )
      )
      my_ahrs.update(delta_ms/1000.0, ahrs_imu_data)
      angles = my_ahrs.get()

      ahrs_packet = packet.Packet('pub', 'ahrs/angle', (angles.pitch, angles.yaw))

      cpu_use_packet = packet.Packet('pub', 'health/os/cpuse', 0.5)
      batt_v_packet = packet.Packet('pub', 'health/batt/v', 12)

      encoded = my_codec.encode(imu_packet) + my_codec.encode(cpu_use_packet) +  my_codec.encode(batt_v_packet) + my_codec.encode(ahrs_packet)

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
    debug.print_exception(e)
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
  run(items[0], items[1], items[2], items[3], items[4], items[5], delta_ms)


