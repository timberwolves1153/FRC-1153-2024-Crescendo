package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BaseClef;
import frc.robot.subsystems.PIDPivot;

public class PivotToAmp extends SequentialCommandGroup{
    
    private PIDPivot pidPivot;
    private BaseClef baseClef;

    public PivotToAmp(PIDPivot pidPivot, BaseClef baseClef) {

        this.pidPivot = pidPivot;
        this.baseClef = baseClef;

        addRequirements(pidPivot);
        addCommands(Commands.runOnce(() -> baseClef.retractClef(), baseClef), 
        new WaitCommand(7),
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(40.2), pidPivot));
    }

   
}
