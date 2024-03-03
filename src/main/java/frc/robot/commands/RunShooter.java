package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

public class RunShooter extends Command{
    private Launcher launcher;
    private WeekZeroVision vision;

    public RunShooter(WeekZeroVision vision, Launcher launcher) {
        this.vision = vision;
        this.launcher = launcher;
    }
    

    @Override
    public void execute() {
        if (vision.calculateRange() > 2) {
            launcher.launchWithVolts();
        } else {
            launcher.closeLaunchSpeed();
        }
    }
}
