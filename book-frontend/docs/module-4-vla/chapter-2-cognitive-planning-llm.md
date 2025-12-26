---
title: "Chapter 2 - Cognitive Planning (LLM → ROS 2)"
sidebar_position: 3
---

# Chapter 2: Cognitive Planning (LLM → ROS 2)

In this chapter, we'll explore how to use large language models (LLMs) for cognitive planning in robotic systems. You'll learn to create intelligent planning systems that can translate high-level goals into executable robotic actions using OpenAI's GPT models.

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate large language models with ROS 2 systems for cognitive planning
- Decompose high-level goals into atomic robotic actions
- Implement context-aware decision making for continuous operation
- Create safety validation systems for LLM-generated plans
- Handle uncertainty and adapt plans based on environmental feedback

## Introduction to LLM-Based Cognitive Planning

Large Language Models (LLMs) excel at understanding natural language and reasoning about complex tasks. In robotics, we can leverage these capabilities to create cognitive planning systems that:

- Understand high-level goals expressed in natural language
- Break down complex tasks into executable steps
- Consider environmental context and constraints
- Generate safe and effective action sequences
- Adapt plans based on real-time feedback

## LLM Integration Framework

Let's start by creating a framework to integrate LLMs with ROS 2:

```python
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

        # Create subscription to receive parsed commands
        self.command_subscription = self.create_subscription(
            String,
            'parsed_commands',
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
```

## Context-Aware Decision Making

To make our system more intelligent, we need to maintain context and state information:

```python
class ContextManager:
    def __init__(self):
        self.world_state = {
            'objects': {},
            'locations': {},
            'robot_state': {},
            'human_interactions': []
        }
        self.memory = []
        self.max_memory_size = 100

    def update_world_state(self, state_update):
        """Update the world state with new information"""
        for key, value in state_update.items():
            if key in self.world_state:
                self.world_state[key].update(value)
            else:
                self.world_state[key] = value

    def add_to_memory(self, event):
        """Add an event to the robot's memory"""
        self.memory.append({
            'timestamp': rclpy.time.Time().nanoseconds,
            'event': event
        })

        # Limit memory size
        if len(self.memory) > self.max_memory_size:
            self.memory = self.memory[-self.max_memory_size:]

    def get_context_prompt(self):
        """Generate a context prompt for the LLM"""
        context = {
            'current_location': self.world_state['robot_state'].get('location', 'unknown'),
            'available_objects': list(self.world_state['objects'].keys()),
            'recent_events': [m['event'] for m in self.memory[-5:]],  # Last 5 events
            'time_of_day': self.get_time_of_day()
        }
        return json.dumps(context, indent=2)

    def get_time_of_day(self):
        """Get the current time of day"""
        import datetime
        current_hour = datetime.datetime.now().hour
        if 5 <= current_hour < 12:
            return 'morning'
        elif 12 <= current_hour < 17:
            return 'afternoon'
        elif 17 <= current_hour < 21:
            return 'evening'
        else:
            return 'night'
```

## Task Decomposition Algorithms

For complex tasks, we need to break them down into smaller, manageable steps:

```python
class TaskDecomposer:
    def __init__(self):
        self.action_library = {
            'find_object': ['locate', 'search_for', 'find'],
            'navigate_to': ['go_to', 'move_to', 'travel_to'],
            'pick_up': ['pick_up', 'grasp', 'take'],
            'place_object': ['place', 'put_down', 'set'],
            'follow_person': ['follow', 'accompany'],
            'wait': ['wait', 'pause', 'stop']
        }

    def decompose_task(self, high_level_task):
        """Decompose a high-level task into atomic actions"""
        # This is a simplified example - in practice, you'd use LLMs for complex decomposition
        if 'bring me a cup from the kitchen' in high_level_task.lower():
            return [
                {'action': 'navigate_to', 'target': 'kitchen'},
                {'action': 'find_object', 'object': 'cup'},
                {'action': 'pick_up', 'object': 'cup'},
                {'action': 'navigate_to', 'target': 'user_location'},
                {'action': 'place_object', 'location': 'near_user'}
            ]
        elif 'go to the living room' in high_level_task.lower():
            return [
                {'action': 'navigate_to', 'target': 'living_room'}
            ]
        else:
            # For unknown tasks, defer to LLM
            return self.decompose_with_llm(high_level_task)

    def decompose_with_llm(self, task):
        """Use LLM to decompose complex tasks"""
        try:
            system_prompt = """
            You are a task decomposition expert. Given a high-level task, break it down into
            atomic robotic actions that can be executed sequentially. Each action should be
            simple and executable. Return a JSON array of action objects.
            """

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Task: {task}"}
                ],
                temperature=0.3,
                max_tokens=300
            )

            response_text = response.choices[0].message.content.strip()
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]

            return json.loads(response_text)
        except Exception as e:
            print(f"Error decomposing task with LLM: {e}")
            return []
```

## Safety and Validation Layer

Safety is crucial when using LLMs for robotic action planning:

```python
class SafetyValidator:
    def __init__(self):
        self.safety_constraints = {
            'forbidden_actions': ['jump', 'fly', 'climb_unsafe', 'enter_restricted_area'],
            'motion_limits': {
                'max_velocity': 1.0,  # m/s
                'max_acceleration': 2.0,  # m/s^2
                'max_joint_velocity': 1.57  # rad/s
            },
            'environment_constraints': {
                'no_go_zones': [],
                'safe_height_range': [0.1, 2.0]  # meters
            }
        }

    def validate_plan(self, action_plan):
        """Validate an action plan for safety"""
        if not action_plan:
            return False, "Empty action plan"

        action_type = action_plan.get('action_type', '').lower()

        # Check for forbidden actions
        if action_type in self.safety_constraints['forbidden_actions']:
            return False, f"Action type '{action_type}' is forbidden"

        # Validate navigation goals
        if action_type == 'navigation':
            x = action_plan.get('x', 0)
            y = action_plan.get('y', 0)

            # Check if destination is in no-go zone
            for zone in self.safety_constraints['environment_constraints']['no_go_zones']:
                if self.is_in_zone(x, y, zone):
                    return False, f"Navigation goal in no-go zone: {zone}"

        # Validate manipulation actions
        elif action_type == 'manipulation':
            object_name = action_plan.get('object', '')
            if not object_name:
                return False, "Manipulation action requires object specification"

        return True, "Plan is valid"

    def is_in_zone(self, x, y, zone):
        """Check if coordinates are within a zone"""
        # Simplified zone checking - in practice, use proper geometric calculations
        center_x, center_y = zone.get('center', (0, 0))
        radius = zone.get('radius', 1.0)
        distance = ((x - center_x)**2 + (y - center_y)**2)**0.5
        return distance <= radius
```

## Memory and Learning System

To enable continuous operation, implement a memory system:

```python
class MemorySystem:
    def __init__(self):
        self.episodic_memory = []  # Memories of specific events
        self.semantic_memory = {}  # General knowledge
        self.procedural_memory = {}  # How-to knowledge
        self.episode_threshold = 1000  # Max number of episodes to remember

    def store_episode(self, task, plan, outcome):
        """Store a complete task execution episode"""
        episode = {
            'timestamp': rclpy.time.Time().nanoseconds,
            'task': task,
            'plan': plan,
            'outcome': outcome,
            'success': outcome.get('success', False)
        }

        self.episodic_memory.append(episode)

        # Maintain memory size
        if len(self.episodic_memory) > self.episode_threshold:
            self.episodic_memory = self.episodic_memory[-self.episode_threshold:]

    def retrieve_similar_episodes(self, current_task):
        """Retrieve similar past episodes to inform current planning"""
        similar_episodes = []
        for episode in self.episodic_memory[-50:]:  # Check last 50 episodes
            if self.tasks_are_similar(current_task, episode['task']):
                similar_episodes.append(episode)
        return similar_episodes

    def tasks_are_similar(self, task1, task2):
        """Determine if two tasks are similar enough to be relevant"""
        # Simple similarity check - in practice, use semantic similarity
        task1_lower = task1.lower()
        task2_lower = task2.lower()

        # Check for common keywords
        common_keywords = ['go', 'get', 'bring', 'move', 'find', 'place']
        for keyword in common_keywords:
            if keyword in task1_lower and keyword in task2_lower:
                return True
        return False

    def update_procedural_knowledge(self, task, successful_plan):
        """Update procedural knowledge based on successful task completion"""
        task_type = self.classify_task(task)
        if task_type not in self.procedural_memory:
            self.procedural_memory[task_type] = []

        self.procedural_memory[task_type].append(successful_plan)
```

## Integration with ROS 2

Now let's put it all together in a comprehensive cognitive planning node:

```python
class IntegratedCognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('integrated_cognitive_planner')

        # Initialize components
        self.context_manager = ContextManager()
        self.task_decomposer = TaskDecomposer()
        self.safety_validator = SafetyValidator()
        self.memory_system = MemorySystem()

        # Initialize OpenAI API
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Create subscribers and publishers
        self.command_subscription = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.action_publisher = self.create_publisher(
            String,
            'low_level_actions',
            10
        )

        self.get_logger().info('Integrated Cognitive Planner initialized')

    def command_callback(self, msg):
        """Process high-level commands through the cognitive planning pipeline"""
        try:
            command = msg.data.strip()
            if not command:
                return

            self.get_logger().info(f'Received high-level command: "{command}"')

            # Retrieve similar past episodes
            similar_episodes = self.memory_system.retrieve_similar_episodes(command)

            # Generate plan using LLM with context
            context = self.context_manager.get_context_prompt()
            action_plan = self.generate_contextual_plan(command, context, similar_episodes)

            if action_plan:
                # Validate the plan for safety
                is_valid, validation_msg = self.safety_validator.validate_plan(action_plan)

                if is_valid:
                    # Execute the plan
                    self.execute_plan(action_plan, command)

                    # Store the episode for future learning
                    outcome = {'success': True, 'details': 'Plan executed successfully'}
                    self.memory_system.store_episode(command, action_plan, outcome)
                else:
                    self.get_logger().warn(f'Safety validation failed: {validation_msg}')
                    # Publish error message or fallback action
                    error_msg = String()
                    error_msg.data = f'Safety validation failed: {validation_msg}'
                    self.action_publisher.publish(error_msg)

        except Exception as e:
            self.get_logger().error(f'Error in cognitive planning: {str(e)}')

    def generate_contextual_plan(self, command, context, similar_episodes):
        """Generate a plan considering context and past experiences"""
        try:
            system_prompt = f"""
            You are an advanced robotic cognitive planner. Consider the following context:

            {context}

            Past successful episodes for similar tasks:
            {json.dumps(similar_episodes[:3])}  # Include up to 3 similar episodes

            Given a high-level command, generate a detailed action plan that:
            1. Considers the current context
            2. Leverages past successful experiences
            3. Is safe and executable
            4. Achieves the specified goal

            Respond with a JSON object containing the action plan.
            """

            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Command: {command}"}
                ],
                temperature=0.3,
                max_tokens=400
            )

            response_text = response.choices[0].message.content.strip()
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]

            return json.loads(response_text)
        except Exception as e:
            self.get_logger().error(f'Error generating contextual plan: {str(e)}')
            return None

    def execute_plan(self, plan, original_command):
        """Execute the generated plan"""
        # Convert plan to executable actions
        action_msg = String()
        action_msg.data = json.dumps({
            'original_command': original_command,
            'plan': plan,
            'timestamp': rclpy.time.Time().nanoseconds
        })

        self.action_publisher.publish(action_msg)
        self.get_logger().info(f'Published plan for execution: {plan}')
```

## Exercises

1. **Integration Exercise**: Implement the full cognitive planning system with context awareness
2. **Optimization Exercise**: Optimize LLM usage to reduce API costs while maintaining planning quality
3. **Safety Exercise**: Implement additional safety checks and validation procedures
4. **Learning Exercise**: Add machine learning capabilities to improve planning based on past experiences

## Summary

In this chapter, you've learned to create sophisticated cognitive planning systems using LLMs. You've implemented context-aware decision making, task decomposition, safety validation, and memory systems. This enables your robot to understand complex goals and generate safe, executable action plans.

In the next chapter, we'll combine all components into a complete autonomous humanoid system.