# OptiTrack Quaternion Fix Documentation
## Problem Solved: Incorrect Yaw Extraction from Quaternions

### The Issue
Your code was using an incorrect formula to extract yaw angle from OptiTrack quaternions. The wrong formula was mixing up quaternion terms, resulting in incorrect robot heading calculations.

### Root Cause
1. **OptiTrack uses Y-up coordinate system**: Y-axis points upward (ceiling), robot moves in XZ plane
2. **Quaternion convention**: OptiTrack sends `[qx, qy, qz, qw]` where `qw` is the real/scalar part
3. **Formula mismatch**: The original formula was for a different coordinate convention

### The Solution

#### Wrong Formula (Original)
```matlab
yaw = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy^2 + qz^2))
```

#### Correct Formula (Verified)
```matlab
yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2))
```

### Test Results
The correct formula was verified with physical robot rotations:
- **90° CCW rotation**: Measured 90.1° (0.1° error) ✓
- **180° rotation**: Measured 179.1° (0.9° error) ✓  
- **90° CW rotation**: Measured -90.5° (0.5° error) ✓

### Files Fixed
1. `calibrate_limo_v2.m` - Line in `getPoseWithYaw()` function
2. `manual_drive_logger.m` - Line where yaw is extracted from quaternion

### Understanding the Coordinate Systems

#### OptiTrack (Y-up)
```
      +Y (up/ceiling)
       |
       |
       |______ +X
      /
     /
    +Z

Robot moves in XZ plane
Yaw = rotation around Y axis
```

#### Your Maze (2D ground plane)
```
   +Y
    |
    |
    |______ +X

Robot moves in XY plane
Heading = angle from +X axis
```

### Coordinate Transform
Your calibration determines how OptiTrack XZ maps to Maze XY:
- Transform 3: `maze_x = optitrack_x`, `maze_y = optitrack_z`
- Transform 4: `maze_x = optitrack_x`, `maze_y = -optitrack_z`

### Complete Euler Extraction (For Reference)
```matlab
function [roll, pitch, yaw] = quaternionToEuler_OptiTrack(qx, qy, qz, qw)
    % For OptiTrack Y-up system (verified by testing)
    
    % Roll (rotation around Z axis in Y-up)
    roll = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
    
    % Pitch (rotation around X axis in Y-up)
    pitch = asin(clamp(2*(qw*qx - qy*qz), -1, 1));
    
    % Yaw (rotation around Y axis in Y-up) - THIS IS WHAT YOU NEED
    yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
end
```

### Key Lessons Learned

1. **Always verify quaternion formulas with physical tests**: Different systems use different conventions
2. **Y-up vs Z-up matters**: Graphics/motion capture often use Y-up, aerospace uses Z-up
3. **Test with known rotations**: 90°, 180°, -90° rotations quickly reveal the correct formula
4. **OptiTrack specifics**:
   - Uses Y-up by default (configurable in Motive)
   - Sends quaternions as `[qx, qy, qz, qw]`
   - Robot yaw is rotation around the vertical (Y) axis

### Quick Debug Check
To verify the fix is working:
1. Place robot pointing "forward"
2. Read yaw angle (should be consistent)
3. Rotate 90° left → yaw should increase by ~90°
4. Rotate 90° right from start → yaw should decrease by ~90°

### Future Prevention
When working with new motion capture or IMU systems:
1. Check coordinate convention (Y-up vs Z-up)
2. Verify quaternion order (scalar first vs last)
3. Test with physical rotations before trusting the math
4. Document the working formula with the system specs

### Additional Notes
- The confusion often comes from mixing aerospace (Z-up) and graphics (Y-up) conventions
- OptiTrack can be configured for different coordinate systems in Motive software
- Always match your extraction formula to the actual coordinate system being used
- Small errors (< 2°) are normal due to manual rotation imprecision
