package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDPivot;

public class MailboxClimbingPosition extends Command{
        private PIDPivot pidPivot;

    public MailboxClimbingPosition(PIDPivot pidPivot) {

        this.pidPivot = pidPivot;
        addRequirements(pidPivot);
    }

    @Override
    public void execute() {
        pidPivot.setSetpointDegrees(60);
    }
}
