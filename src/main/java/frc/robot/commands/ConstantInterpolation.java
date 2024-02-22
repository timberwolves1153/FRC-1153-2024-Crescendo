package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDPivot;

public class ConstantInterpolation extends Command{
    
    private PIDPivot pidPivot;
    private BooleanSupplier pivotWithVision;

    public ConstantInterpolation(PIDPivot pidPivot, BooleanSupplier pivotWithVision) {

        this.pidPivot = pidPivot;
        this.pivotWithVision = pivotWithVision;
        addRequirements(pidPivot);
    }

    @Override
    public void execute() {
        if (pivotWithVision.getAsBoolean()) {
        pidPivot.interpolateSetpoint();
    } else {
        pidPivot.setSetpointDegrees(14.5);;
    }
}

@Override
public void end(boolean interrupted) {
    pidPivot.setSetpointDegrees(14.5);
}


}
