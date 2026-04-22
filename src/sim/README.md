# sim package

## Nodes

- `map_publisher`: Reads `map.csv` as binary grid and publishes `nav_msgs/OccupancyGrid` on `/map`
- `map_visualizer`: Subscribes to `/map` and displays it using pygame (black=occupied, white=free)

## CSV Format
`src/sim/map.csv` must contain a **2D binary grid**:
- Each line = one row of the map
- Comma-separated values: `0` = free, `1` = occupied
- All rows must have identical length

**Example** (`15x15`):
```
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,1,1,0,1,1,1,0,1,1,1,0,1,0,1
1,0,0,0,0,0,0,0,0,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,0,0,0,0,0,1
1,1,1,0,1,1,1,0,1,1,1,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,0,0,0,1,0,0,0,1,0,0,0,1,0,1
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
```

**Result**: OccupancyGrid with `height=15`, `width=15`, resolution=`0.05m`, origin=`(0,0)`.

## Usage
```bash
colcon build --packages-select sim
source install/setup.bash
ros2 run sim map_publisher &
ros2 run sim map_visualizer
```

Close the pygame window to exit visualizer. Map is published every 5 seconds.