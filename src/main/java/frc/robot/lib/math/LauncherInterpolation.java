package frc.robot.lib.math;

import frc.robot.lib.Interpolation.InterpolatingDouble;
import frc.robot.lib.Interpolation.InterpolatingTreeMap;

public class LauncherInterpolation {
    

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap = new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = new InterpolatingTreeMap<>();

// key = distance from speaker meters
// value = rpm setpoint
    static {    
        pivotMap.put(new InterpolatingDouble(1.44), new InterpolatingDouble(56.0));
        pivotMap.put(new InterpolatingDouble(1.85), new InterpolatingDouble(50.5));
        pivotMap.put(new InterpolatingDouble(2.207), new InterpolatingDouble(48.0));
        pivotMap.put(new InterpolatingDouble(2.52), new InterpolatingDouble(39.4));
        pivotMap.put(new InterpolatingDouble(2.84), new InterpolatingDouble(35.9));
        pivotMap.put(new InterpolatingDouble(3.04), new InterpolatingDouble(34.6));
        pivotMap.put(new InterpolatingDouble(3.42), new InterpolatingDouble(30.0));
        pivotMap.put(new InterpolatingDouble(3.83), new InterpolatingDouble(27.7));
        pivotMap.put(new InterpolatingDouble(4.21), new InterpolatingDouble(26.0));
        pivotMap.put(new InterpolatingDouble(4.52), new InterpolatingDouble(23.7));
        pivotMap.put(new InterpolatingDouble(4.68), new InterpolatingDouble(23.6));
        pivotMap.put(new InterpolatingDouble(5.12), new InterpolatingDouble(22.7));
        pivotMap.put(new InterpolatingDouble(5.25), new InterpolatingDouble(22.5));
        pivotMap.put(new InterpolatingDouble(5.58), new InterpolatingDouble(22.3));
    }
}
