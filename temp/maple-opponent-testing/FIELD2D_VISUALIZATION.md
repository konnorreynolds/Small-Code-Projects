# Field2d Obstacle Visualization Guide

This document shows what you'll see in the Field2d widget during simulation.

## Field Layout (Reefscape 2025)

```
Field: 17.548m x 8.052m

  8.0m ┌─────────────────────────────────────────────────────────────┐
       │ TOP WALL ═══════════════════════════════════════════════════│
       │                                                             │
  7.0m │                                                             │
       │                                                             │
       │                                                             │
  6.0m │                                                             │
       │                                                             │
       │                                                             │
  5.0m │                                                             │
       │                                                             │
       │          ⚫ Blue        ⚫ Center       ⚫ Red               │
  4.0m │  🤖          Reef         Pillar         Reef       🎯 GOAL │
       │ START     (4.5,4)       (8.8,4)       (13.1,4)     (14,4)  │
  3.0m │  (3,3)                                                      │
       │                                                             │
       │                     🔴 Opponent0                            │
  2.0m │                                                             │
       │                                 🔴 Opponent1                │
       │                                                             │
  1.0m │                                                             │
       │                                                             │
       │ BOTTOM WALL ════════════════════════════════════════════════│
     0m└─────────────────────────────────────────────────────────────┘
       0m   2    4     6     8    10    12    14    16   17.548m
       │                                                             │
     LEFT                                                         RIGHT
     WALL                                                         WALL
```

## Field2d Objects

### Robot (Blue)
- **Name**: Robot pose
- **Starting Position**: (3, 3, 0°)
- **Color**: Blue
- **Updates**: Every 20ms in `periodic()`

### Opponents (Red)
- **Name**: "Opponent0", "Opponent1"
- **Type**: KitBot (differential drive)
- **Color**: Red
- **Updates**: Every 20ms in `periodic()`
- **Movement**: Dynamic - they move around the field

### Goal (Green)
- **Name**: "Goal"
- **Position**: (14, 4, 0°)
- **Color**: Green
- **Updates**: When A button is pressed

### Static Obstacles

#### Walls (4 total)
Visualized as lines of robot icons:

1. **Obstacle0_wall** (Left Wall)
   - Position: x=0, y=0 to 8.052
   - Points: ~16 poses vertically
   - Orientation: 0°

2. **Obstacle1_wall** (Right Wall)
   - Position: x=17.548, y=0 to 8.052
   - Points: ~16 poses vertically
   - Orientation: 0°

3. **Obstacle2_wall** (Bottom Wall)
   - Position: y=0, x=0 to 17.548
   - Points: ~35 poses horizontally
   - Orientation: 90°

4. **Obstacle3_wall** (Top Wall)
   - Position: y=8.052, x=0 to 17.548
   - Points: ~35 poses horizontally
   - Orientation: 90°

#### Circular Obstacles (3 total)

5. **Obstacle4 & Obstacle4_circle** (Blue Reef)
   - Center: (4.489, 4.026)
   - Radius: 0.8m
   - Visualization: 16 poses in circle
   - Each pose oriented tangent to circle

6. **Obstacle5 & Obstacle5_circle** (Red Reef)
   - Center: (13.059, 4.026)
   - Radius: 0.8m
   - Visualization: 16 poses in circle
   - Each pose oriented tangent to circle

7. **Obstacle6 & Obstacle6_circle** (Center Pillar)
   - Center: (8.774, 4.026)
   - Radius: 0.4m
   - Visualization: 16 poses in circle
   - Each pose oriented tangent to circle

## What You'll See in Sim GUI

### Initial State (Robot Enabled)
```
┌─────────────────────────────────────────┐
│ Field2d Widget                          │
├─────────────────────────────────────────┤
│                                         │
│  🤖 You (3,3)                           │
│                                         │
│      ⚫⚫⚫  Blue Reef                    │
│                                         │
│               ⚫ Pillar                  │
│                                         │
│                      ⚫⚫⚫  Red Reef     │
│                                         │
│  🔴 Opponent0         🔴 Opponent1      │
│                                         │
│  [Walls shown as lines along edges]    │
└─────────────────────────────────────────┘
```

### When A Button is Pressed
```
┌─────────────────────────────────────────┐
│ Field2d Widget                          │
├─────────────────────────────────────────┤
│                                         │
│  🤖 ─ ─ ─ ─ ─>                          │
│ (3,3)    Path curves                    │
│      ⚫⚫⚫  around obstacles             │
│            ↗                            │
│               ⚫  ↗                      │
│                 ↗                       │
│                ↗  ⚫⚫⚫    🎯 (14,4)     │
│               ↗                 Goal    │
│  🔴          ↗     🔴                   │
│  Opponent0 ↗      Opponent1             │
│                                         │
│  [Robot follows smooth arc]             │
└─────────────────────────────────────────┘
```

### Expected Path Behavior

The robot path should:
1. **Start**: (3, 3) heading east
2. **Curve upward**: Avoid Blue Reef at (4.5, 4)
3. **Navigate around**: Avoid opponents dynamically
4. **Stay centered**: Between top and bottom walls
5. **Avoid pillar**: Pass above or below center pillar
6. **Final approach**: Straight line to (14, 4) after clearing obstacles

## SmartDashboard Values During Navigation

```
Obstacles/Total: 9
  ├─ Static: 7 (4 walls + 3 circles)
  └─ Dynamic: 2 (opponents)

Obstacles/Dynamic: 2

Distance to Goal:
  ├─ Start: ~11.0 meters
  ├─ Mid-path: ~7.5 meters
  └─ Arrival: <0.1 meters

Obstacle Count: "Static: 7, Dynamic: 2"
```

## Color Legend (Standard Field2d Colors)

- **Blue Robot Icons**: Your robot
- **Red Robot Icons**: Opponent robots
- **Green Robot Icons**: Goal position
- **Gray/White Robot Icons**: Static obstacles (walls, reefs, pillar)

## Tips for Best Visualization

1. **Resize Field2d Widget**: Make it larger for better view
2. **Clear Other Objects**: Hide trajectory if too cluttered
3. **Zoom In**: Focus on obstacle-rich areas (center of field)
4. **Watch Opponents**: They update in real-time as they move
5. **Path Trace**: Enable robot trajectory to see full path history

## Troubleshooting Visualization

### Obstacles Don't Appear
- Check console for "Visualized X static obstacles on Field2d"
- Verify Field2d widget is showing in Glass/Shuffleboard
- Ensure `SmartDashboard.putData("Field", field)` is called

### Circles Look Weird
- This is normal - circles are approximated with 16 poses
- Each pose shows a robot icon oriented tangent to circle
- The pattern should still clearly show circular shape

### Walls Overlap
- Walls extend slightly into field (0.3m safety margin)
- This is intentional for conservative avoidance
- Robot should stay well clear of wall obstacles

### Goal Doesn't Show When A is Pressed
- Make sure A button is HELD, not just tapped
- Check that `driveToPose()` command is running
- Verify Field2d object "Goal" is created

## Advanced: Viewing in Glass vs Shuffleboard

### Glass (Recommended)
- Cleaner interface
- Better performance with many objects
- Right-click Field2d → Configure → Set colors

### Shuffleboard
- More widgets available
- Can see SmartDashboard values alongside field
- May be slower with 20+ objects on field

## Example Session Output

```
Console Output:
--------------
Created 7 Reefscape static obstacles
Visualized 7 static obstacles on Field2d

Field2d Objects:
--------------
- Robot (blue)
- Opponent0 (red)
- Opponent1 (red)
- Goal (green) - when A pressed
- Obstacle0_wall (left)
- Obstacle1_wall (right)
- Obstacle2_wall (bottom)
- Obstacle3_wall (top)
- Obstacle4 + Obstacle4_circle (blue reef)
- Obstacle5 + Obstacle5_circle (red reef)
- Obstacle6 + Obstacle6_circle (center pillar)

Total: 17 Field2d objects
```

## What Makes This Effective

1. **Clear Obstacle Boundaries**: Walls shown as continuous lines
2. **Circular Patterns**: Reefs and pillar clearly identifiable
3. **Real-Time Dynamics**: Opponents update continuously
4. **Goal Indication**: Clear target for navigation
5. **Path Visualization**: Robot leaves trail showing avoidance behavior

The visualization makes it immediately obvious if the robot is:
- Avoiding obstacles correctly
- Taking efficient paths
- Responding to dynamic opponents
- Reaching the goal successfully
