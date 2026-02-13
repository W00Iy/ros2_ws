from pid_controller_msgs.srv import SetReference

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult


class jointSimulator:
    def __init__(self):
        # Krav i oppgaven:
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.voltage = 0.0
        self.noise = 0.0  # foreløpig 0
        self.reference = 0.0  # <-- referansen som service skal oppdatere

        # Konstanter:
        self.K = 230.0
        self.T = 0.15

    def update(self, dt: float):
        # Overføringsfunksjon
        V = self.voltage + self.noise
        omega_dot = (self.K * V - self.angular_velocity) / self.T

        self.angular_velocity += dt * omega_dot
        self.angle += dt * self.angular_velocity


class JointSimulatorNode(Node):
    def __init__(self):
        # Hvis du MÅ ha nodenavn "pid_controller_node" for oppgaven:
        # super().__init__("pid_controller_node")
        # Hvis du vil beholde simulator-navnet:
        super().__init__("joint_simulator_node")

        self.sim = jointSimulator()

        # -----------------------------
        # SERVICE (OPPGAVEN)
        # -----------------------------
        self.srv = self.create_service(
            SetReference,
            "/pid_controller_node/set_reference",
            self.set_reference_callback
        )
        self.get_logger().info("Service ready: /pid_controller_node/set_reference")

        # 1) Deklarer parametere (default-verdier)
        self.declare_parameter("noise", 0.0)
        self.declare_parameter("K", 230.0)
        self.declare_parameter("T", 0.15)

        # 2) Les parametere ved oppstart og sett i simulatoren
        self._apply_parameters_from_server()

        # 3) Callback: gjør at ros2 param set virker mens noden kjører
        self.add_on_set_parameters_callback(self.parameter_callback)

        # publisher som heter publish_angle (publiserer Float64 med angle)
        self.publish_angle = self.create_publisher(Float64, "/angle", 10)

        # subscriber som heter input_voltage og callback voltage_listener
        self.input_voltage = self.create_subscription(
            Float64, "/voltage", self.voltage_listener, 10
        )

        # wall_timer som kaller update + publish
        self.dt = 0.01  # 10 ms
        self.timer = self.create_timer(self.dt, self.timer_callback)

    # -----------------------------
    # Service callback (OPPGAVEN!)
    # -----------------------------
    def set_reference_callback(self, request, response):
        ref = float(request.reference)

        # Sjekk at verdien er gyldig og innen [-pi, pi]
        if math.isfinite(ref) and (-math.pi <= ref <= math.pi):
            # Oppdater referansen (her lagrer vi i simulatoren)
            self.sim.reference = ref
            response.success = True
            self.get_logger().info(f"Ny referanse satt til {ref:.3f} rad")
        else:
            response.success = False
            self.get_logger().warn(
                f"Ugyldig referanse {ref:.3f} (må være mellom -pi og pi)"
            )

        return response

    def _apply_parameters_from_server(self):
        self.sim.noise = float(self.get_parameter("noise").value)
        self.sim.K = float(self.get_parameter("K").value)
        self.sim.T = float(self.get_parameter("T").value)

        self.get_logger().info(
            f"Params: K={self.sim.K}, T={self.sim.T}, noise={self.sim.noise}"
        )

    def parameter_callback(self, params):
        for p in params:
            if p.name == "noise":
                self.sim.noise = float(p.value)

            elif p.name == "K":
                self.sim.K = float(p.value)

            elif p.name == "T":
                new_T = float(p.value)
                if new_T <= 0.0:
                    return SetParametersResult(successful=False, reason="T must be > 0")
                self.sim.T = new_T

        return SetParametersResult(successful=True)

    def voltage_listener(self, msg: Float64):
        self.sim.voltage = float(msg.data)

    def timer_callback(self):
        self.sim.update(self.dt)

        msg = Float64()
        msg.data = float(self.sim.angle)
        self.publish_angle.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JointSimulatorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
