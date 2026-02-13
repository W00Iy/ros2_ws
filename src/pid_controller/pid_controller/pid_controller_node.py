#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64

from pid_controller_msgs.srv import SetReference


class pidController:
    """
    Enkel PID-kontroller med:
      - p, i, d, reference, voltage
      - integrator + derivertledd
      - dt-beregning med tid
      - metning (clamp) og enkel anti-windup
    """

    def __init__(self, p: float, i: float, d: float, reference: float = 0.0):
        self.p = float(p)
        self.i = float(i)
        self.d = float(d)

        self.reference = float(reference)
        self.voltage = 0.0

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

        self.min_voltage = -12.0
        self.max_voltage = 12.0
        self.integral_limit = 100.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self.voltage = 0.0

    def set_reference(self, reference: float):
        self.reference = float(reference)

    def update(self, measurement: float) -> float:
        now = time.time()
        dt = 0.0 if self._prev_time is None else (now - self._prev_time)

        error = self.reference - float(measurement)

        # Integrator
        if dt > 0.0:
            self._integral += error * dt
            self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        # Derivertledd
        derivative = 0.0 if dt <= 0.0 else (error - self._prev_error) / dt

        # PID-sum
        u = (self.p * error) + (self.i * self._integral) + (self.d * derivative)

        # Metning + enkel anti-windup
        u_clamped = max(self.min_voltage, min(self.max_voltage, u))
        if u != u_clamped and dt > 0.0:
            self._integral *= 0.98

        self.voltage = float(u_clamped)

        self._prev_error = error
        self._prev_time = now

        return self.voltage


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Parametre
        self.declare_parameter('p', 1.0)
        self.declare_parameter('i', 0.0)
        self.declare_parameter('d', 0.0)
        self.declare_parameter('reference', 0.0)

        p = self.get_parameter('p').value
        i = self.get_parameter('i').value
        d = self.get_parameter('d').value
        reference = self.get_parameter('reference').value

        self.pid = pidController(p=p, i=i, d=d, reference=reference)

        # Dynamisk parameteroppdatering
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Pub/Sub
        self.publisher_ = self.create_publisher(Float64, 'voltage', 10)
        self.subscription_ = self.create_subscription(
            Float64,
            'angle',
            self.measurement_listener,
            10
        )

        # SERVICE: set_reference
        self.srv_ = self.create_service(
            SetReference,
            'set_reference',
            self.set_reference_callback
        )

        self.get_logger().info(
            f"Start: p={p:.3f}, i={i:.3f}, d={d:.3f}, reference={reference:.3f}"
        )
        self.get_logger().info("Service tilgjengelig: /set_reference (pid_controller_msgs/srv/SetReference)")

    def publish_voltage(self):
        msg = Float64()
        msg.data = float(self.pid.voltage)
        self.publisher_.publish(msg)

    def measurement_listener(self, msg: Float64):
        measured = float(msg.data)
        self.pid.update(measured)
        self.publish_voltage()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p':
                if float(param.value) >= 0.0:
                    self.pid.p = float(param.value)
                    self.get_logger().info(f"p satt til: {self.pid.p}")
                else:
                    return SetParametersResult(successful=False, reason="p må være >= 0")

            elif param.name == 'i':
                if float(param.value) >= 0.0:
                    self.pid.i = float(param.value)
                    self.get_logger().info(f"i satt til: {self.pid.i}")
                else:
                    return SetParametersResult(successful=False, reason="i må være >= 0")

            elif param.name == 'd':
                if float(param.value) >= 0.0:
                    self.pid.d = float(param.value)
                    self.get_logger().info(f"d satt til: {self.pid.d}")
                else:
                    return SetParametersResult(successful=False, reason="d må være >= 0")

            elif param.name == 'reference':
                self.pid.reference = float(param.value)
                self.get_logger().info(f"reference satt til: {self.pid.reference}")

        return SetParametersResult(successful=True)

    def set_reference_callback(self, request, response):
        ref = float(request.request)  # matcher .srv: float64 request

        if -math.pi <= ref <= math.pi:
            self.pid.set_reference(ref)

            # hold parameteren i sync (så ros2 param get viser riktig)
            try:
                self.set_parameters([Parameter('reference', Parameter.Type.DOUBLE, ref)])
            except Exception:
                pass

            self.get_logger().info(f"set_reference: reference satt til {ref:.3f}")
            response.success = True
        else:
            self.get_logger().warn(
                f"set_reference: ugyldig reference {ref:.3f} (må være [-pi, pi])"
            )
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
