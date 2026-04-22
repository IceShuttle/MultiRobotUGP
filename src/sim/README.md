# sim package

## Nodes

- `map_publisher`: Reads `map.csv` (grid of 0/1) → publishes `/map`
- `entity_sim`: Subscribes to `/map`, randomly places 5 humans + 3 fires on free cells → publishes `/entities` (MarkerArray for RViz)
- `map_visualizer`: Shows map + humans (blue circles) + fires (red circles) in pygame

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