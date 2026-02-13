#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pid_controller_msgs.srv import SetReference


class ReferenceInputNode(Node):
    def __init__(self):
        super().__init__('reference_input_node')
        self.cli = self.create_client(SetReference, '/set_reference')

        self.get_logger().info("Venter på /set_reference ...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service ikke tilgjengelig, venter...")

        self.get_logger().info("Klar! Skriv inn ny referanse.")

    def send_reference(self, value: float) -> None:
        req = SetReference.Request()
        req.request = float(value)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"Servicekall feilet: {future.exception()}")
            return

        if future.result().success:
            self.get_logger().info(f"OK: sendte referanse {value}")
        else:
            self.get_logger().warning(f"FEIL: controller avviste referanse {value}")


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceInputNode()

    try:
        while rclpy.ok():
            user_in = input("Sett inn ny referanse: ")
            try:
                node.send_reference(float(user_in))
            except ValueError:
                node.get_logger().error("Ugyldig input. Skriv et tall (f.eks. 2 eller 2.5).")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
