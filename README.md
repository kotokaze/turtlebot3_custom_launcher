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
