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
        pivotMap.put(new InterpolatingDouble(2.207), new InterpolatingDouble(48.0));
        pivotMap.put(new InterpolatingDouble(2.7), new InterpolatingDouble(37.7));
        pivotMap.put(new InterpolatingDouble(3.04), new InterpolatingDouble(34.6));
        pivotMap.put(new InterpolatingDouble(3.42), new InterpolatingDouble(30.0));
        pivotMap.put(new InterpolatingDouble(3.83), new InterpolatingDouble(27.7));

        pivotMap.put(new InterpolatingDouble(4.19), new InterpolatingDouble(25.3));
        pivotMap.put(new InterpolatingDouble(5.12), new InterpolatingDouble(22.13));
        pivotMap.put(new InterpolatingDouble(5.58), new InterpolatingDouble(21.6));
    }
}
