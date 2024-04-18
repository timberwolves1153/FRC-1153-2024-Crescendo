package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BaseClef;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

public class UpdateLauncher extends Command {
    
    private PIDPivot pidPivot;
    private WeekZeroVision vision;

    public UpdateLauncher(PIDPivot pidPivot, WeekZeroVision vision) {

        this.pidPivot = pidPivot;
        this.vision = vision;

        addRequirements(pidPivot);
        addRequirements(vision);
        pidPivot.interpolateSetpoint();
    }

   
}
