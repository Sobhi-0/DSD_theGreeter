from rclpy.node import Node


class HomeAssistantNode(Node):
    def __init__(self):
        super().__init__("homeassistant")
        self.get_logger().info("HomeAssistantNode has been started.")


def main():
    print("Hi from homeassitant.")


if __name__ == "__main__":
    main()
