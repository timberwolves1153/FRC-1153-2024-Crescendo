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
        pivotMap.put(new InterpolatingDouble(1.85), new InterpolatingDouble(52.5));
        pivotMap.put(new InterpolatingDouble(2.207), new InterpolatingDouble(45.8));
        pivotMap.put(new InterpolatingDouble(2.6), new InterpolatingDouble(37.0));
        pivotMap.put(new InterpolatingDouble(2.72), new InterpolatingDouble(35.9));
        pivotMap.put(new InterpolatingDouble(3.04), new InterpolatingDouble(32.0));
        pivotMap.put(new InterpolatingDouble(3.42), new InterpolatingDouble(27.0));
        pivotMap.put(new InterpolatingDouble(3.83), new InterpolatingDouble(25.0));
        pivotMap.put(new InterpolatingDouble(4.21), new InterpolatingDouble(24.0));
        pivotMap.put(new InterpolatingDouble(4.68), new InterpolatingDouble(23.0));
        pivotMap.put(new InterpolatingDouble(5.12), new InterpolatingDouble(20.5));
        pivotMap.put(new InterpolatingDouble(5.2), new InterpolatingDouble(19.6));
        // pivotMap.put(new InterpolatingDouble(5.58), new InterpolatingDouble(22.3));
    }
}
