#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from service_interfaces.msg import GetTouchAct1


class HandControlSubscriber(Node):
    def __init__(self):
        super().__init__("handcontrol_subscriber")

        spacename = str(self.declare_parameter("spacename", "inspire").value).strip("/")
        hand_name = str(self.declare_parameter("name", "hand1").value).strip("/")
        topic_prefix = "/".join(part for part in (spacename, hand_name) if part)
        touch_topic = f"{topic_prefix}/touch_data" if topic_prefix else "touch_data"

        self.subscription = self.create_subscription(
            GetTouchAct1,
            touch_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"订阅话题: {touch_topic}")

    def listener_callback(self, msg):
        lines = ["触觉数据 (法向力 / 切向力):"]
        for i, fid in enumerate(msg.finger_ids):
            name = msg.finger_names[i] if i < len(msg.finger_names) else ""
            nf = msg.normal_forces[i] if i < len(msg.normal_forces) else 0.0
            tf = msg.tangential_forces[i] if i < len(msg.tangential_forces) else 0.0
            lines.append(f"  {name} (id={fid}): 法向力={nf:.4f}, 切向力={tf:.4f}")
        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    subscriber = HandControlSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("手动停止节点")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
