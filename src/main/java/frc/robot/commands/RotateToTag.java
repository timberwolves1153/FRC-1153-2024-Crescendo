package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.VisionExperiment;

public class RotateToTag extends Command{
        private Swerve swerve;
        private VisionExperiment vision;

    public RotateToTag(Swerve swerve, VisionExperiment vision) {

        this.swerve = swerve;
        this.vision = vision;

    }

    @Override
    public void execute() {
        vision.aimAtTarget();
    }
}
