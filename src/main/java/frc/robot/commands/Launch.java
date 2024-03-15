package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

public class Launch extends Command{
    private Launcher launcher;
    private WeekZeroVision vision;
    
    public Launch(Launcher launcher, WeekZeroVision vision) {

        this.launcher= launcher;
        this.vision = vision;
    }

    @Override
    public void execute() {
        if(vision.calculateRange() > 4) {
            launcher.launchAtWing();
        } else if (vision.calculateRange() < 2) {
            launcher.closeLaunchSpeed();
        } else {
            launcher.launchWithVolts();
        }
    }

    
}
