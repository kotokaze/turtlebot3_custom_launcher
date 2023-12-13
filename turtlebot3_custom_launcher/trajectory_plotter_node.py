#!/usr/bin/env python3.10
from pathlib import Path
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor as Executor
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

from matplotlib import pyplot as plt


class TrajectoryPlotterNode(Node):
  """
  Subscribes to the model states topic and log the trajectories of the robots.

  """
  def __init__(self) -> None:
    super().__init__('trajectory_plotter_node')

    self.declare_parameter('model_states_topic',
      '/gazebo/model_states',
      ParameterDescriptor(
        type=rclpy.Parameter.Type.STRING,
        description='The topic to subscribe to for model states.',
      ),
    )

    self.create_timer(
      1,  # Run every 1 sec
      self._timer_callback,
    )

    self.create_subscription(
        ModelStates,
        self.get_parameter('model_states_topic').get_parameter_value().string_value,
        self._model_state_callback,
        10,
    )

    self.robot_trajectories: dict[str, list[Point]] = {}
    self.lines: dict[str, 'plt.Line2D'] = {}

    self.fig, self.ax = plt.subplots()
    self.ax.set_title('Robot Trajectories')
    self.ax.set_xlabel('x-axis')
    self.ax.set_ylabel('y-axis')
    self.ax.grid(True)
    self.ax.set_xlim(-3, 3)
    self.ax.set_ylim(-0.2, 0.2)

  def save_file(self) -> None:
    # Create a directory with the time stamp
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    Path(now).mkdir(parents=True, exist_ok=True)

    for name, trajectory in self.robot_trajectories.items():
      with Path(f'{now}').joinpath(f'{name}.csv').open('w+', encoding='utf8') as f:
        f.write('x,y\n')
        for pose in trajectory:
          f.write(f'{pose.x},{pose.y}\n')

  def _model_state_callback(self, msg: ModelStates) -> None:
    for name, pose in zip(msg.name, msg.pose):
      if name.startswith('tb3_'):
        self.get_logger().debug(f'{name}: {pose.position}')
        self.robot_trajectories.setdefault(name, []).append(pose.position)

        if name not in self.lines:
          self.lines[name], = self.ax.plot([], [], label=name)

  def _timer_callback(self) -> None:

    all_x: list[float] = []
    all_y: list[float] = []

    for name, line in self.lines.items():
      x = [pose.x for pose in self.robot_trajectories[name]]
      y = [pose.y for pose in self.robot_trajectories[name]]
      self.get_logger().debug(f'{name}: {x}, {y}')

      all_x.extend(x)
      all_y.extend(y)
      line.set_data(x, y)

    self.ax.legend()

    if all_x:
      max_all_x = max(all_x)
      min_all_x = min(all_x)
      if 6 < ( max_all_x - min_all_x):
        self.ax.set_xlim(min(all_x), max(all_x))

    if all_y:
      max_all_y = max(all_y)
      min_all_y = min(all_y)
      if 0.4 < (max_all_y - min_all_y):
        self.ax.set_ylim(min(all_y), max(all_y))

    self.fig.canvas.draw()
    self.fig.canvas.flush_events()
    plt.pause(0.001)  # Update the figure with the new data.


def main(args: Optional[list[str]] = None) -> None:
  """
  Main function to initialize and run the trajectory plotter node.

  Args:
    args (Optional[list[str]]): Command-line arguments (default: None).
  """
  rclpy.init(args=args)
  executor = Executor()

  node = TrajectoryPlotterNode()
  executor.add_node(node)

  try:
    executor.spin()
  except KeyboardInterrupt:
    node.save_file()
    executor.shutdown()
    rclpy.shutdown()
