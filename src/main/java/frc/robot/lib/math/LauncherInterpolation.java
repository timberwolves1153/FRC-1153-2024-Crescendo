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
        pivotMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(37.0));
        pivotMap.put(new InterpolatingDouble(2.21), new InterpolatingDouble(32.0));
        pivotMap.put(new InterpolatingDouble(2.53), new InterpolatingDouble(27.0));
        pivotMap.put(new InterpolatingDouble(2.68), new InterpolatingDouble(25.0));
        pivotMap.put(new InterpolatingDouble(2.97), new InterpolatingDouble(21.5));
        pivotMap.put(new InterpolatingDouble(3.09), new InterpolatingDouble(21.0));
    }
}
