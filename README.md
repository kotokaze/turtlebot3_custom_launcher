# TurtleBot3 Custom Launcher
## Usage
### Simulation

With custom args:

```bash
ros2 launch turtlebot3_custom_launcher simulation.launch.py \
  world_name:=empty_world.world \
  num_robots:=3
```

> [!TIP]  
> You can find other the available args via  
> > `ros2 launch turtlebot3_custom_launcher simulation.launch.py --show-args`


### Trajectory Plotter Node

You can change the monitor topic name with `model_states_topic:=<topic_name>`

```bash
ros2 run turtlebot3_custom_launcher trajectory_plotter_node
```

> [!IMPORTANT]  
> `model_states` topic must be published with the `gazebo_ros_state` plugin.  
> You can clone my fork: [kotokaze/turtlebot3_simulations](https://github.com/kotokaze/turtlebot3_simulations) to the `src` directory of your workspace and build it.
> > See [this commit](https://github.com/kotokaze/turtlebot3_simulations/commit/6125faa15e8feb58c0191ade22ce12fedaa20370?diff=split&w=1) for the changes.
