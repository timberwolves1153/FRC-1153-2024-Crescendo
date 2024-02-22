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
        pivotMap.put(new InterpolatingDouble(2.05), new InterpolatingDouble(40.0));
        pivotMap.put(new InterpolatingDouble(2.22), new InterpolatingDouble(34.5));
        pivotMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(30.0));
        pivotMap.put(new InterpolatingDouble(2.76), new InterpolatingDouble(26.0));
        pivotMap.put(new InterpolatingDouble(2.97), new InterpolatingDouble(23.8));
        pivotMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(22.8));
    }
}
