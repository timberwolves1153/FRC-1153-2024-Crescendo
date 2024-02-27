package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDPivot;

public class InterpolateToSpeaker extends Command{
    private PIDPivot pidPivot;

    public InterpolateToSpeaker(PIDPivot pidPivot) {

        this.pidPivot = pidPivot;
        addRequirements(pidPivot);
    }

    @Override
    public void execute() {
        pidPivot.interpolateSetpoint();
    }
}
