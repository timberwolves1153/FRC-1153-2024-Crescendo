package frc.robot.lib.math;

import frc.robot.lib.Interpolation.InterpolatingDouble;
import frc.robot.lib.Interpolation.InterpolatingTreeMap;

public class LauncherInterpolation {
    

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap = new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = new InterpolatingTreeMap<>();

// key = distance from speaker meters
// value = rpm setpoint
    static {    
        launcherMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(5000.0));
    }
}
