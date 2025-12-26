import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')

        # Create subscription to raw text commands
        self.raw_command_subscription = self.create_subscription(
            String,
            'raw_commands',
            self.raw_command_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Create publisher for parsed commands
        self.parsed_command_publisher = self.create_publisher(
            String,
            'parsed_commands',
            10
        )

        # Define command vocabulary and patterns
        self.command_patterns = {
            'move': ['go to', 'move to', 'navigate to', 'walk to', 'travel to'],
            'pick': ['pick up', 'grab', 'take', 'collect', 'get'],
            'place': ['place', 'put', 'drop', 'set down'],
            'follow': ['follow', 'come with me', 'accompany'],
            'stop': ['stop', 'halt', 'pause', 'wait'],
            'greet': ['hello', 'hi', 'greet', 'say hello', 'wave']
        }

        self.get_logger().info('Command Parser node initialized')

    def raw_command_callback(self, msg):
        """Parse raw commands and convert to structured format"""
        try:
            raw_command = msg.data.lower().strip()
            if not raw_command:
                return

            # Parse the command
            parsed_command = self.parse_command(raw_command)

            if parsed_command:
                # Publish parsed command
                parsed_msg = String()
                parsed_msg.data = json.dumps(parsed_command)
                self.parsed_command_publisher.publish(parsed_msg)

                self.get_logger().info(f'Parsed command: {parsed_command}')
        except Exception as e:
            self.get_logger().error(f'Error parsing command: {str(e)}')

    def parse_command(self, command):
        """Parse a natural language command into structured format"""
        command_lower = command.lower()

        # Identify command type
        command_type = None
        for cmd_type, patterns in self.command_patterns.items():
            if any(pattern in command_lower for pattern in patterns):
                command_type = cmd_type
                break

        if not command_type:
            return None

        # Extract additional information
        result = {
            'command_type': command_type,
            'original_command': command,
            'confidence': 0.9  # High confidence for simple pattern matching
        }

        # Add specific parameters based on command type
        if command_type == 'move':
            # Extract destination if mentioned
            if 'kitchen' in command_lower:
                result['destination'] = 'kitchen'
                result['coordinates'] = {'x': 1.0, 'y': 2.0, 'z': 0.0}
            elif 'living room' in command_lower:
                result['destination'] = 'living_room'
                result['coordinates'] = {'x': 3.0, 'y': 1.0, 'z': 0.0}
            elif 'bedroom' in command_lower:
                result['destination'] = 'bedroom'
                result['coordinates'] = {'x': 0.0, 'y': -1.0, 'z': 0.0}
            else:
                result['destination'] = 'unknown'

        elif command_type in ['pick', 'place']:
            # Extract object if mentioned
            if 'cup' in command_lower:
                result['object'] = 'cup'
            elif 'book' in command_lower:
                result['object'] = 'book'
            elif 'ball' in command_lower:
                result['object'] = 'ball'
            else:
                result['object'] = 'unknown'

        return result


def main(args=None):
    rclpy.init(args=args)

    command_parser_node = CommandParserNode()

    try:
        rclpy.spin(command_parser_node)
    except KeyboardInterrupt:
        pass
    finally:
        command_parser_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()