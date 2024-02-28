package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.AprilTags.WeekZeroVision;
    

public class ShootWhenReady extends Command{

    private WeekZeroVision vision;
    private PIDPivot pidPivot;
    private Launcher launcher;
    private Mailbox mailbox;

    public ShootWhenReady() {
        this.vision = vision;
        this.launcher = launcher;
        this.pidPivot = pidPivot;
        this.mailbox = mailbox;

    }

    @Override
    public void execute() {
        if (Math.abs(vision.aimAtTarget()) < 1 
        && pidPivot.getPivotRadians() == pidPivot.getSetpoint() 
        && launcher.getLeftLauncherAppliedVolts() > 9.5 
        && launcher.getRightLauncherAppliedVolts() > 6) {

            mailbox.sendToLauncher();
        } else {
            mailbox.stop();
        }
    }
}
