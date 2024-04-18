package frc.robot.lib.math;

import java.util.Map;
import java.util.function.DoubleSupplier;

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
        pivotMap.put(new InterpolatingDouble(1.55), new InterpolatingDouble(51.0));
        pivotMap.put(new InterpolatingDouble(1.65), new InterpolatingDouble(48.0));
        pivotMap.put(new InterpolatingDouble(1.75), new InterpolatingDouble(46.6));
        pivotMap.put(new InterpolatingDouble(1.85), new InterpolatingDouble(44.5));
        pivotMap.put(new InterpolatingDouble(1.95), new InterpolatingDouble(42.5));
        pivotMap.put(new InterpolatingDouble(2.05), new InterpolatingDouble(41.0));
        pivotMap.put(new InterpolatingDouble(2.15), new InterpolatingDouble(37.5));
        pivotMap.put(new InterpolatingDouble(2.25), new InterpolatingDouble(36.7));
        pivotMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(36.29));
        pivotMap.put(new InterpolatingDouble(2.45), new InterpolatingDouble(34.71));
        pivotMap.put(new InterpolatingDouble(2.55), new InterpolatingDouble(34.4));
        pivotMap.put(new InterpolatingDouble(2.65), new InterpolatingDouble(33.13));
        pivotMap.put(new InterpolatingDouble(2.75), new InterpolatingDouble(32.2));
        pivotMap.put(new InterpolatingDouble(2.85), new InterpolatingDouble(30.93));
        pivotMap.put(new InterpolatingDouble(2.95), new InterpolatingDouble(29.6));
        pivotMap.put(new InterpolatingDouble(3.05), new InterpolatingDouble(29.1));
        pivotMap.put(new InterpolatingDouble(3.15), new InterpolatingDouble(28.7));
        pivotMap.put(new InterpolatingDouble(3.25), new InterpolatingDouble(27.9));
        pivotMap.put(new InterpolatingDouble(3.35), new InterpolatingDouble(27.4));
        pivotMap.put(new InterpolatingDouble(3.45), new InterpolatingDouble(26.5));
        pivotMap.put(new InterpolatingDouble(3.55), new InterpolatingDouble(25.7));
        pivotMap.put(new InterpolatingDouble(3.65), new InterpolatingDouble(24.8));
        pivotMap.put(new InterpolatingDouble(3.75), new InterpolatingDouble(24.7));
        pivotMap.put(new InterpolatingDouble(3.85), new InterpolatingDouble(24.6));
        pivotMap.put(new InterpolatingDouble(3.95), new InterpolatingDouble(24.5));
        pivotMap.put(new InterpolatingDouble(4.05), new InterpolatingDouble(24.2));
        pivotMap.put(new InterpolatingDouble(4.15), new InterpolatingDouble(23.85));
        pivotMap.put(new InterpolatingDouble(4.25), new InterpolatingDouble(23.55));
        pivotMap.put(new InterpolatingDouble(4.35), new InterpolatingDouble(23.3));
        pivotMap.put(new InterpolatingDouble(4.45), new InterpolatingDouble(22.9));
        pivotMap.put(new InterpolatingDouble(4.55), new InterpolatingDouble(22.45));
        pivotMap.put(new InterpolatingDouble(4.65), new InterpolatingDouble(22.2));

        // pivotMap.put(new InterpolatingDouble(2.61), new InterpolatingDouble(32.5));
        // pivotMap.put(new InterpolatingDouble(2.72), new InterpolatingDouble(30.4));
        // pivotMap.put(new InterpolatingDouble(3.06), new InterpolatingDouble(27.2));
        // pivotMap.put(new InterpolatingDouble(3.33), new InterpolatingDouble(27.3));
        // pivotMap.put(new InterpolatingDouble(3.55), new InterpolatingDouble(24.2));
        // pivotMap.put(new InterpolatingDouble(3.8), new InterpolatingDouble(23.0));
        // pivotMap.put(new InterpolatingDouble(4.01), new InterpolatingDouble(21.1));
        // pivotMap.put(new InterpolatingDouble(4.22), new InterpolatingDouble(20.4));
        // pivotMap.put(new InterpolatingDouble(4.45), new InterpolatingDouble(20.1));
        // pivotMap.put(new InterpolatingDouble(4.6), new InterpolatingDouble(19.1));
        // pivotMap.put(new InterpolatingDouble(4.7), new InterpolatingDouble(19.1));
        // pivotMap.put(new InterpolatingDouble(5.1), new InterpolatingDouble(19.0));
        // pivotMap.put(new InterpolatingDouble(5.2), new InterpolatingDouble(18.8));
    }

    public DoubleSupplier getSetpointSupplier(double distance) {
        return () -> pivotMap.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    public double getSetpoint(double distance) {
        return pivotMap.getInterpolated(new InterpolatingDouble(distance)).value;
    }
}
