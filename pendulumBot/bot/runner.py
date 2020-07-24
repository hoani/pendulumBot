from pendulumBot.comms import btServer, tcpServer

import time, sys, os
from pendulumBot.bot.robotControl import *
from pendulumBot.bot import motorPair, ahrs
from pendulumBot.comms import commandRegister, commandCallbacks, remoteLogger
from pendulumBot.utilities import vect, imuData, cli, debug
from pendulumBot.control import pid, pendulum 

import leap

import datetime

class RobotRunner:
  def __init__(self, settings):
    args = cli.get_args(settings["default"])
    simulate = args.simulate

    self.codec = leap.Codec('config/protocol.json')
    self.cpu_usage = 0.0
    self.delta_max_s = 0.0
    self.max_cpu_usage = 0.0
    self.battery_v = 0.0

    hostBtMACAddress = args.bluetooth[0]
    hostBtPort = args.bluetooth[1]

    hostTcpIpAddress = args.tcp[0]
    hostTcpPortCommand = int(args.tcp[1])
    hostTcpPortLogging = int(args.tcp[2])

    if (simulate):
      from pendulumBot.driver.simulated import motors, imu, adc, servo
      self.adc = adc
      self.servo = servo
      self.rcpy = None

    else:
      import rcpy
      from pendulumBot.driver.rcpy import motors, imu, adc, servo
      rcpy.set_state(rcpy.RUNNING)
      self.adc = adc
      self.servo = servo
      self.rcpy = rcpy

    self.sockets = []

    try:

      left = motors.dcMotor(3)
      right = motors.dcMotor(2)

      pair = motorPair.MotorPair(left, right)

      if (simulate == False):
        self.sockets.append(btServer.btServer(hostBtMACAddress, hostBtPort))
      self.sockets.append(tcpServer.TcpServer(hostTcpIpAddress, hostTcpPortCommand))

      self.logger_socket = tcpServer.TcpServer(hostTcpIpAddress, hostTcpPortLogging)
      self._remote_logger_setup()

      self.robo = RobotControl(pair)
      self.imu = imu.Imu(
        mapping = vect.Vec3(2,0,1),
        sign = vect.Vec3(-1,-1,1)
      )
      self.pitch_pid = pid.Pid(kp = 1.0, ki = 0.0, kd = 0.0, setpoint = 0.0)
      self.ahrs = ahrs.AhrsTwoWheeled()

      self.pendulum_control = pendulum.PendulumPid(self.pitch_pid, self.ahrs, limit = 5.0)

      self.robo.add_controller(RobotControl.STATE_PENDULUM, self.pendulum_control)

      self.servo_set = [
        self.servo(1), self.servo(2), self.servo(3), self.servo(4),
        self.servo(5), self.servo(6), self.servo(7), self.servo(8)
      ]

      self.callback_objs = [
        commandCallbacks.PendulumCallbacks(self.robo, self.pendulum_control),
        commandCallbacks.RobotControlCallbacks(self.robo),
        commandCallbacks.RemoteLogCallbacks(self.remote_logger),
        commandCallbacks.AhrsCallbacks(self.ahrs),
        commandCallbacks.ServoCallbacks(self.servo_set)
      ]

      self.registry = commandRegister.CommandRegister()
      for obj in self.callback_objs:
        obj.register(self.registry)

    except Exception as e:
      debug.print_exception(e)
      for sock in self.sockets:
        sock.close()

    finally:
      pair.stop()


  def run(self, update_period_ms):
    delta_ms = update_period_ms
    delta_max_ms = 0
    last_data = "".encode("utf-8")
    next_ms = (datetime.datetime.now().timestamp() * 1000.0) + update_period_ms
    publish_period_ms = 200
    current_publish_ms = 0
    last_publish_ms = 0
    try:
      while self.check_exit_conditions():
        last_s = time.time()
        self.robo.update(delta_ms)

        # comms_update(sockets)
        self.logger_socket.accept_connections()
        _ = self.logger_socket.recv() # Only used to determine if a connection has dropped

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
                response = leap.Packet("ack")
                
                for cmd in p.paths:
                  response.add(cmd)

                unpacked = p.unpack(self.codec)

                for path in unpacked.keys():
                  if self.registry.execute(path, unpacked[path]) == False:
                    print("Command {} Failed".format(path))
                    response.category = "nak"

              sock.send(addr, self.codec.encode(response))

        # publish_subscribed_data(codec, sockets)
        imu_data = self.imu.sample()
        self.ahrs.update(delta_ms/1000.0, imu_data)
        angles = self.ahrs.get()

        current_publish_ms += delta_ms

        if current_publish_ms - last_publish_ms >= publish_period_ms:
          last_publish_ms += publish_period_ms

          self.battery_v = self.adc.battery_voltage()
          self._publish_subscribed_packets(imu_data, angles)

        self._remote_logger_update(imu_data, angles, delta_ms)
        

        # Note, this must be done at the end of a step
        self._calculate_cpu_usage(last_s, update_period_ms * 0.001)

        # Calculate timing
        delta_ms = self._rest_until(next_ms, update_period_ms)
        next_ms += delta_ms


    except Exception as e:
      debug.print_exception(e)
      for sock in self.sockets:
        sock.close()

    finally:
      self.robo.wheels.stop()

  def check_exit_conditions(self):
    if self.rcpy == None:
      return True
    return self.rcpy.get_state() != self.rcpy.EXITING

  def _calculate_cpu_usage(self, last_s, update_period_s):
    delta_meas_s = time.time() - last_s
    self.delta_max_s = max(delta_meas_s, self.delta_max_s)

    self.cpu_usage = 100.0 * (delta_meas_s / update_period_s)
    self.max_cpu_usage = max(self.max_cpu_usage, self.cpu_usage)

    print(('\r'
          '|| overhead (s) = {:0.3f} | cpu usage (%) = {:0.1f} |'
          '| max overhead (s) = {:0.3f} | max usage (%) = {:0.1f} ||'
          ).format(delta_meas_s, self.cpu_usage, self.delta_max_s, self.max_cpu_usage), end='')

  def _rest_until(self, next_ms, update_period_ms):
    current_ms = datetime.datetime.now().timestamp()*1000.0
    sleep_ms = next_ms - current_ms
    if sleep_ms > 0.0:
      time.sleep(sleep_ms * 0.001)
      delta_ms = update_period_ms
    else:
      delta_ms = 0.0
      while(sleep_ms + delta_ms < 0):
        delta_ms += update_period_ms

    return int(delta_ms)

  def _publish_subscribed_packets(self, imu_data, angles):
    imu_packet = leap.Packet('pub', 'imu',
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
    ahrs_packet = leap.Packet('pub', 'ahrs/angle', (angles.pitch, angles.yaw))
    cpu_use_packet = leap.Packet('pub', 'health/os/cpuse', self.cpu_usage)
    batt_v_packet = leap.Packet('pub', 'health/batt/v', self.battery_v)
    motor_packet = leap.Packet('pub', 'motor', self.robo.get_wheels_duty())

    encoded = (
      self.codec.encode(imu_packet) +
      self.codec.encode(cpu_use_packet) +
      self.codec.encode(batt_v_packet) +
      self.codec.encode(ahrs_packet) +
      self.codec.encode(motor_packet)
    )

    for sock in self.sockets:
      for addr in sock.get_clients():
        sock.send(addr, encoded)


  def _remote_logger_setup(self):
    self.remote_logger = remoteLogger.RemoteLogger(self.logger_socket)
    self.remote_logger.add("timeMs")

    self.remote_logger.add("imuAccX")
    self.remote_logger.add("imuAccY")
    self.remote_logger.add("imuAccZ")

    self.remote_logger.add("imuGyrX")
    self.remote_logger.add("imuGyrY")
    self.remote_logger.add("imuGyrZ")

    self.remote_logger.add("imuMagX")
    self.remote_logger.add("imuMagY")
    self.remote_logger.add("imuMagZ")

    self.remote_logger.add("ahrsPitch")
    self.remote_logger.add("ahrsYaw")

    self.remote_logger.add("motorL")
    self.remote_logger.add("motorR")

    self.remote_logger.add("cpu%")
    self.remote_logger.add("battV")


  def _remote_logger_update(self, imu_data, angles, delta_ms):
    self.remote_logger.set("timeMs", "{:d}".format(self.remote_logger.current_ms))
    self.remote_logger.set("imuAccX", "{:0.03f}".format(imu_data.accelerometer.x))
    self.remote_logger.set("imuAccY", "{:0.03f}".format(imu_data.accelerometer.y))
    self.remote_logger.set("imuAccZ", "{:0.03f}".format(imu_data.accelerometer.z))
    self.remote_logger.set("imuGyrX", "{:0.02f}".format(imu_data.gyroscope.x))
    self.remote_logger.set("imuGyrY", "{:0.02f}".format(imu_data.gyroscope.x))
    self.remote_logger.set("imuGyrZ", "{:0.02f}".format(imu_data.gyroscope.x))
    self.remote_logger.set("imuMagX", "{:0.02f}".format(imu_data.magnetometer.x))
    self.remote_logger.set("imuMagY", "{:0.02f}".format(imu_data.magnetometer.x))
    self.remote_logger.set("imuMagZ", "{:0.02f}".format(imu_data.magnetometer.x))
    self.remote_logger.set("ahrsPitch", "{:0.02}".format(angles.pitch))
    self.remote_logger.set("ahrsYaw", "{:0.02}".format(angles.yaw))
    self.remote_logger.set("motorL", "{:0.02}".format(self.robo.get_wheels_duty()[0]))
    self.remote_logger.set("motorR", "{:0.02}".format(self.robo.get_wheels_duty()[1]))
    self.remote_logger.set("cpu%", "{:0.02}".format(self.cpu_usage))
    self.remote_logger.set("battV", "{:0.03}".format(self.battery_v))

    self.remote_logger.update(delta_ms)

  