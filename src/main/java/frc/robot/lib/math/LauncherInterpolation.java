package frc.robot.lib.math;

import frc.robot.lib.Interpolation.InterpolatingDouble;
import frc.robot.lib.Interpolation.InterpolatingTreeMap;

public class LauncherInterpolation {
    

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap = new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = new InterpolatingTreeMap<>();
    private static final double PIGEON_OFFSET = 180;

// key = distance from speaker meters
// value = rpm setpoint
    static {    
        pivotMap.put(new InterpolatingDouble(1.44), new InterpolatingDouble(57.0));
        pivotMap.put(new InterpolatingDouble(1.85), new InterpolatingDouble(48.5));
        pivotMap.put(new InterpolatingDouble(2.227), new InterpolatingDouble(38.9));
        pivotMap.put(new InterpolatingDouble(2.61), new InterpolatingDouble(34.5));
        pivotMap.put(new InterpolatingDouble(2.72), new InterpolatingDouble(32.8));
        pivotMap.put(new InterpolatingDouble(3.06), new InterpolatingDouble(29.7));
        pivotMap.put(new InterpolatingDouble(3.33), new InterpolatingDouble(27.3));
        pivotMap.put(new InterpolatingDouble(3.55), new InterpolatingDouble(25.0));
        pivotMap.put(new InterpolatingDouble(3.8), new InterpolatingDouble(24.3));
        pivotMap.put(new InterpolatingDouble(4.01), new InterpolatingDouble(23.7));
        pivotMap.put(new InterpolatingDouble(4.22), new InterpolatingDouble(22.5));
        pivotMap.put(new InterpolatingDouble(4.45), new InterpolatingDouble(21.5));
        pivotMap.put(new InterpolatingDouble(4.6), new InterpolatingDouble(21.3));
        pivotMap.put(new InterpolatingDouble(4.7), new InterpolatingDouble(20.5));
        pivotMap.put(new InterpolatingDouble(5.1), new InterpolatingDouble(20.0));
        pivotMap.put(new InterpolatingDouble(5.2), new InterpolatingDouble(18.8));
    }
}
