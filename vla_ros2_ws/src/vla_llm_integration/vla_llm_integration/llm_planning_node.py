import rclpy
from rclpy.node import Node
import openai
import os
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Initialize OpenAI API key from environment variable
        openai.api_key = os.getenv('OPENAI_API_KEY')
        if not openai.api_key:
            self.get_logger().warn('OPENAI_API_KEY environment variable not set!')

        # Create subscription to receive recognized text commands
        self.command_subscription = self.create_subscription(
            String,
            'recognized_text',
            self.command_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Create publishers for different action types
        self.navigation_publisher = self.create_publisher(
            Pose,
            'navigation_goal',
            10
        )

        self.action_publisher = self.create_publisher(
            String,
            'robot_action',
            10
        )

        self.get_logger().info('LLM Planning node initialized')

    def command_callback(self, msg):
        """Process natural language commands and generate action plans"""
        try:
            command = msg.data.strip()
            if not command:
                return

            self.get_logger().info(f'Received command: "{command}"')

            # Generate action plan using LLM
            action_plan = self.generate_action_plan(command)

            if action_plan:
                self.execute_action_plan(action_plan)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def generate_action_plan(self, command):
        """Generate an action plan using the LLM"""
        try:
            # Define the system prompt for the planner
            system_prompt = """
            You are a robotic action planner. Given a natural language command,
            break it down into executable robotic actions. Respond with a JSON
            object containing the action type and parameters.

            Possible action types:
            - navigation: Move to a specific location
            - manipulation: Manipulate objects
            - interaction: Interact with humans or objects
            - monitoring: Monitor the environment

            For navigation, include x, y, z coordinates and orientation (quaternion).
            For other actions, include relevant parameters.
            """

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Command: {command}"}
                ],
                temperature=0.3,
                max_tokens=200
            )

            # Extract and parse the response
            response_text = response.choices[0].message.content.strip()

            # Clean up the response to extract JSON
            if response_text.startswith('```json'):
                response_text = response_text[7:]  # Remove ```json
            if response_text.endswith('```'):
                response_text = response_text[:-3]  # Remove ```

            action_plan = json.loads(response_text)
            self.get_logger().info(f'Generated action plan: {action_plan}')

            return action_plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response as JSON: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error generating action plan: {str(e)}')
            return None

    def execute_action_plan(self, action_plan):
        """Execute the action plan by publishing to appropriate topics"""
        action_type = action_plan.get('action_type', '').lower()

        if action_type == 'navigation':
            # Create and publish navigation goal
            pose_msg = Pose()
            pose_msg.position.x = action_plan.get('x', 0.0)
            pose_msg.position.y = action_plan.get('y', 0.0)
            pose_msg.position.z = action_plan.get('z', 0.0)

            # Set default orientation if not provided
            pose_msg.orientation.x = action_plan.get('orientation', {}).get('x', 0.0)
            pose_msg.orientation.y = action_plan.get('orientation', {}).get('y', 0.0)
            pose_msg.orientation.z = action_plan.get('orientation', {}).get('z', 0.0)
            pose_msg.orientation.w = action_plan.get('orientation', {}).get('w', 1.0)

            self.navigation_publisher.publish(pose_msg)
            self.get_logger().info(f'Published navigation goal: {pose_msg}')

        elif action_type in ['manipulation', 'interaction', 'monitoring']:
            # Publish generic action command
            action_msg = String()
            action_msg.data = json.dumps(action_plan)
            self.action_publisher.publish(action_msg)
            self.get_logger().info(f'Published action command: {action_msg.data}')

        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')


def main(args=None):
    rclpy.init(args=args)

    llm_planning_node = LLMPlanningNode()

    try:
        rclpy.spin(llm_planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_planning_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()