package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PIDPivot;

public class PivotToAmp extends Command{
    
    private PIDPivot pidPivot;
    public PivotToAmp(PIDPivot pidPivot) {

        this.pidPivot = pidPivot;

        addRequirements(pidPivot);
    }

    @Override
    public void execute() {
         pidPivot.setSetpointDegrees(49.7);
    }
}
