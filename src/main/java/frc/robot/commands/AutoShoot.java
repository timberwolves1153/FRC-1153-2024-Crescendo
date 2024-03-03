package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

public class AutoShoot extends Command{
    private Launcher launcher;
    private PIDPivot pidPivot;
    private WeekZeroVision vision;
    private Mailbox mailbox;
    private WaitCommand waitCommand;

    public AutoShoot(Launcher launcher, PIDPivot pidPivot, Mailbox mailbox, WeekZeroVision vision) {
        
        this.launcher = launcher;
        this.pidPivot = pidPivot;
        this.mailbox = mailbox;
        this.vision = vision;
    }

    @Override
    public void execute() {

        if (vision.calculateRange() > 2) {
                if (launcher.isLauncherReadyToShootFar() && pidPivot.isPivotReadyToShoot() && Math.abs(vision.aimAtTarget()) < 1) {
                
                    new WaitCommand(0.5);
                    mailbox.sendToLauncher();

            } else {
                mailbox.stop();
            }
        } else if (vision.calculateRange() <= 2){

            if (launcher.isLauncherReadyToShootClose() && pidPivot.isPivotReadyToShoot() && Math.abs(vision.aimAtTarget()) < 1) {
                
                    new WaitCommand(0.5);
                    mailbox.sendToLauncher();
            } else {
                mailbox.stop();
            }
    }
    }
}
