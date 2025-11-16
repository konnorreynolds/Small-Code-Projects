# Obstacle Avoidance Tuning Guide

All tunable parameters are at the **top of `SwerveSubsystem.java`** (lines 53-72).

## Quick Tuning Scenarios

### 🐌 Conservative/Safe Navigation
```java
private static final double NAV_SPEED_MPS = 3.0;        // Slow
private static final double AVOIDANCE_RADIUS = 2.0;     // Large safety margin
private static final double GOAL_ATTRACTION = 1.5;      // Weak goal pull
private static final double WALL_WEIGHT = 4.0;          // Strong wall avoidance
```
**Use when:** Testing, narrow spaces, many obstacles

### ⚡ Aggressive/Fast Navigation
```java
private static final double NAV_SPEED_MPS = 7.0;        // Fast
private static final double AVOIDANCE_RADIUS = 1.0;     // Tight margins
private static final double GOAL_ATTRACTION = 4.0;      // Strong goal pull
private static final double WALL_WEIGHT = 2.0;          // Lighter wall avoidance
```
**Use when:** Open field, few obstacles, time-critical

### 🎯 Balanced (Default)
```java
private static final double NAV_SPEED_MPS = 5.0;
private static final double AVOIDANCE_RADIUS = 1.5;
private static final double GOAL_ATTRACTION = 2.5;
private static final double WALL_WEIGHT = 3.0;
```
**Use when:** General navigation, moderate obstacle density

## Parameter Reference

### Navigation Speed
| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `NAV_SPEED_MPS` | 5.0 | 2.0-8.0 | Robot speed during obstacle avoidance |

**Higher = faster but less time to react**
**Lower = safer but slower**

### Obstacle Sizes
| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `WALL_RADIUS` | 0.4 | 0.3-0.6 | Size of each wall obstacle circle |
| `WALL_SPACING` | 0.6 | 0.4-1.0 | Gap between wall circles |
| `REEF_RADIUS` | 0.95 | 0.7-1.2 | Size of reef obstacles |

**Larger radius = stronger avoidance but less navigable space**
**Smaller spacing = denser wall coverage, harder to slip through**

### Avoidance Strength
| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `WALL_WEIGHT` | 3.0 | 1.5-5.0 | Wall repulsion force multiplier |
| `REEF_WEIGHT` | 2.5 | 1.0-4.0 | Reef repulsion force multiplier |
| `OPPONENT_WEIGHT` | 1.2 | 0.8-2.0 | Opponent avoidance multiplier |

**Higher weight = stronger repulsion, wider berth around obstacles**

### APF Behavior
| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `AVOIDANCE_RADIUS` | 1.5 | 0.8-2.5 | Safety margin around all obstacles |
| `AVOIDANCE_STRENGTH` | 1.8 | 1.0-3.0 | Base repulsion force strength |
| `GOAL_ATTRACTION` | 2.5 | 1.0-5.0 | Pull toward goal |
| `PREDICTION_TIME` | 1.5 | 0.5-3.0 | Collision prediction look-ahead |

**AVOIDANCE_RADIUS:** Distance at which robot starts avoiding
**AVOIDANCE_STRENGTH:** How hard obstacles push robot away
**GOAL_ATTRACTION:** How hard goal pulls robot (balance with avoidance)
**PREDICTION_TIME:** How far ahead robot predicts collisions

## Common Issues & Solutions

### 🚫 Robot drives through obstacles
**Problem:** Avoidance forces too weak
**Solution:**
```java
private static final double WALL_WEIGHT = 4.0;          // Increase from 3.0
private static final double AVOIDANCE_STRENGTH = 2.5;   // Increase from 1.8
private static final double AVOIDANCE_RADIUS = 2.0;     // Increase from 1.5
```

### 🐌 Robot moves too slowly / gets stuck
**Problem:** Repulsion forces overwhelming goal attraction
**Solution:**
```java
private static final double GOAL_ATTRACTION = 4.0;      // Increase from 2.5
private static final double WALL_WEIGHT = 2.0;          // Decrease from 3.0
private static final double NAV_SPEED_MPS = 6.0;        // Increase from 5.0
```

### 🎯 Robot can't reach goal near obstacles
**Problem:** Safety margins too large for tight spaces
**Solution:**
```java
private static final double AVOIDANCE_RADIUS = 1.0;     // Decrease from 1.5
private static final double REEF_RADIUS = 0.8;          // Decrease from 0.95
private static final double WALL_SPACING = 0.8;         // Increase from 0.6
```

### ⚡ Robot swerves/oscillates
**Problem:** Forces unbalanced or prediction time too long
**Solution:**
```java
private static final double PREDICTION_TIME = 1.0;      // Decrease from 1.5
private static final double NAV_SPEED_MPS = 4.0;        // Decrease from 5.0
```

## Testing Strategy

### Step 1: Verify Wall Avoidance
1. Set goal to (14, 4) - crosses entire field
2. Robot should stay within boundaries
3. If it hits walls, increase `WALL_WEIGHT`

### Step 2: Test Reef Avoidance
1. Robot should curve around blue reef at (4.5, 4)
2. If it collides, increase `REEF_WEIGHT` or `REEF_RADIUS`

### Step 3: Test Opponent Avoidance
1. Enable teleop to activate opponents
2. Robot should navigate around moving opponents
3. If it collides, increase `OPPONENT_WEIGHT`

### Step 4: Optimize Speed
1. Gradually increase `NAV_SPEED_MPS`
2. Find maximum speed that still avoids obstacles
3. Typical range: 4-6 m/s

## Formula Relationships

**Total Repulsion Force = AVOIDANCE_STRENGTH × WEIGHT × (1 / distance²)**
**Effective Safety Zone = AVOIDANCE_RADIUS + OBSTACLE_RADIUS**

### Force Balance
- If `GOAL_ATTRACTION` >> `AVOIDANCE_STRENGTH × WALL_WEIGHT`: Robot ignores obstacles
- If `AVOIDANCE_STRENGTH × WALL_WEIGHT` >> `GOAL_ATTRACTION`: Robot can't reach goal
- **Ideal:** `GOAL_ATTRACTION ≈ 0.5 × AVOIDANCE_STRENGTH × WALL_WEIGHT`

## Advanced Tuning

### For Dynamic Obstacles (Opponents)
```java
private static final double PREDICTION_TIME = 2.0;      // Longer for faster opponents
private static final double OPPONENT_WEIGHT = 1.5;      // Higher for aggressive avoidance
```

### For Tight Spaces
```java
private static final double WALL_SPACING = 0.4;         // Denser coverage
private static final double WALL_RADIUS = 0.3;          // Smaller obstacles
private static final double GOAL_ATTRACTION = 3.5;      // Stronger pull through gaps
```

### For Open Field
```java
private static final double WALL_SPACING = 1.0;         // Sparse coverage
private static final double NAV_SPEED_MPS = 7.0;        // High speed
private static final double AVOIDANCE_RADIUS = 1.0;     // Tight margins
```

## Example Configurations

### Competition Mode (Balanced)
```java
private static final double NAV_SPEED_MPS = 5.5;
private static final double WALL_WEIGHT = 3.0;
private static final double REEF_WEIGHT = 2.5;
private static final double OPPONENT_WEIGHT = 1.3;
private static final double AVOIDANCE_RADIUS = 1.4;
private static final double GOAL_ATTRACTION = 2.8;
```

### Testing Mode (Ultra-Safe)
```java
private static final double NAV_SPEED_MPS = 3.0;
private static final double WALL_WEIGHT = 4.5;
private static final double REEF_WEIGHT = 3.5;
private static final double OPPONENT_WEIGHT = 2.0;
private static final double AVOIDANCE_RADIUS = 2.0;
private static final double GOAL_ATTRACTION = 1.5;
```

### Speed Run Mode (Risky)
```java
private static final double NAV_SPEED_MPS = 8.0;
private static final double WALL_WEIGHT = 2.0;
private static final double REEF_WEIGHT = 1.8;
private static final double OPPONENT_WEIGHT = 0.8;
private static final double AVOIDANCE_RADIUS = 1.0;
private static final double GOAL_ATTRACTION = 5.0;
```
