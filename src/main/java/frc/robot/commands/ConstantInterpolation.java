package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDPivot;

public class ConstantInterpolation extends Command{
    
    private PIDPivot pidPivot;
    private BooleanSupplier pivotToSpeaker;
    private BooleanSupplier pivotUpOverride;
    private BooleanSupplier pivotDownOverride;
    private BooleanSupplier togglerOverride;
    private BooleanSupplier pivotToAmp;

    public ConstantInterpolation(PIDPivot pidPivot, BooleanSupplier pivotToSpeaker, BooleanSupplier pivotToAmp, BooleanSupplier pivotUpOverride, BooleanSupplier  pivotDownOverride, BooleanSupplier toggleOverride) {

        this.pidPivot = pidPivot;
        this.pivotToSpeaker = pivotToSpeaker;
        this.pivotToAmp = pivotToAmp;
        this.pivotUpOverride = pivotUpOverride;
        this.pivotDownOverride = pivotDownOverride;
        this.togglerOverride = toggleOverride;
        addRequirements(pidPivot);
    }

    @Override
    public void execute() {
        if (pivotToSpeaker.getAsBoolean()) {
        pidPivot.interpolateSetpoint();
    } else if (pivotToAmp.getAsBoolean()) {
        pidPivot.setSetpointDegrees(56);
    } else if (pivotUpOverride.getAsBoolean() && togglerOverride.getAsBoolean()) {
        pidPivot.pivotUp();
    } else if (pivotDownOverride.getAsBoolean() && togglerOverride.getAsBoolean()) {
        pidPivot.pivotDown();
    } else if (togglerOverride.getAsBoolean()) {
        pidPivot.holdPosition();
    } else {
        pidPivot.setSetpointDegrees(20);
    }
}

@Override
public void end(boolean interrupted) {
    pidPivot.setSetpointDegrees(20);
}



}
