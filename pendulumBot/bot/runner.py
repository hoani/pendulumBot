from pendulumBot.comms import btServer, tcpServer

import time, sys, os
from pendulumBot.bot.robotControl import *
from pendulumBot.bot import motorPair, ahrs
from pendulumBot.comms import commandRegister, commandCallbacks
from pendulumBot.utilities import vect, imuData, cli, debug

from external.RoBus.RoBus import codec, packet

import datetime



class RobotRunner:
  def __init__(self, settings):
    args = cli.get_args(settings["default"])
    simulate = args.simulate

    self.codec = codec.Codec('config/protocol.json')

    hostBtMACAddress = args.bluetooth[0]
    hostBtPort = args.bluetooth[1]

    hostTcpIpAddress = args.tcp[0]
    hostTcpPortCommand = int(args.tcp[1])
    hostTcpPortLogging = int(args.tcp[2])

    if (simulate):
      from pendulumBot.driver.simulated import motors, imu
      self.check_exit_conditions = self._check_exit_conditions_simulated

    else:
      import rcpy
      from pendulumBot.driver.rcpy import motors, imu
      rcpy.set_state(rcpy.RUNNING)
      self.check_exit_conditions = self._check_exit_conditions_rcpy

    self.sockets = []

    try:

      left = motors.dcMotor(3)
      right = motors.dcMotor(2)

      pair = motorPair.MotorPair(left, right)

      if (simulate == False):
        self.sockets.append(btServer.btServer(hostBtMACAddress, hostBtPort))
      self.sockets.append(tcpServer.TcpServer(hostTcpIpAddress, hostTcpPortCommand))

      self.robo = RobotControl(pair)
      self.imu = imu.Imu()
      self.ahrs = ahrs.AhrsTwoWheeled() 

      self.rc_callbacks = commandCallbacks.RobotControlCallbacks(self.robo)

      self.registry = commandRegister.CommandRegister()
      self.rc_callbacks.register(self.registry)


    except Exception as e:
      debug.print_exception(e)
      for sock in self.sockets:
        sock.close()

    finally:
      pair.stop()


  def run(self, delta_ms):
    delta_max_ms = 0
    last_data = "".encode("utf-8")
    try:
      while self.check_exit_conditions():
        # TODO: replace sleep timing with interval timing
        last_ms = datetime.datetime.now().timestamp()
        self.robo.update(delta_ms)

        # comms_update(sockets)
        for sock in self.sockets:
          sock.accept_connections()
          rx = sock.recv()
          if rx != None:
            # process_packets(rx, sock)
            (addr, data) = rx
            print("\nReceived: {}".format(data))

            (last_data, packets) = self.codec.decode(last_data + data)

            # To do - respond NAK to a group with an unknown command

            for p in packets:
              if p.category == "set":
                response = packet.Packet("ack")
                for (cmd, payload) in tuple(zip(p.paths, p.payloads)):
                  response.add(cmd)
                  if commands.execute(cmd, payload) == False:
                    print("Command {} Failed", cmd)
                    response.category = "nak"

              sock.send(addr, self.codec.encode(response))

        # publish_subscribed_data(codec, sockets)
        imu_data = self.imu.sample()

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
        self.ahrs.update(delta_ms/1000.0, ahrs_imu_data)
        angles = self.ahrs.get()

        ahrs_packet = packet.Packet('pub', 'ahrs/angle', (angles.pitch, angles.yaw))

        cpu_use_packet = packet.Packet('pub', 'health/os/cpuse', 0.5)
        batt_v_packet = packet.Packet('pub', 'health/batt/v', 12)

        encoded = (
          self.codec.encode(imu_packet) + 
          self.codec.encode(cpu_use_packet) +  
          self.codec.encode(batt_v_packet) + 
          self.codec.encode(ahrs_packet)
        )

        for sock in self.sockets:
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
      for sock in self.sockets:
        sock.close()

    finally:
      self.robo.wheels.stop()

  def _check_exit_conditions_rcpy(self):
    return rcpy.get_state() != rcpy.EXITING

  def _check_exit_conditions_simulated(self):
    return True