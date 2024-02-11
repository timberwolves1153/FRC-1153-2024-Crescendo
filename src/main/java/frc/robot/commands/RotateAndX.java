package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RotateAndX extends Command {
    
    private Swerve swerve;
    private PIDController thetaController;
    private double rotationVal;

    public RotateAndX(Swerve swerve) {
        this.swerve = swerve;
        thetaController = new PIDController(0.1, 0, 0);

    addRequirements(swerve);        
    }

    @Override
    public void initialize() {
        thetaController.enableContinuousInput(-180, 180);
        thetaController.setTolerance(0);
    }

    @Override
    public void execute() {
            thetaController.setSetpoint(45);

            rotationVal = thetaController.calculate((MathUtil.inputModulus(swerve.getPose().getRotation().getDegrees(), -180, 180)), thetaController.getSetpoint());
            rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxAngularVelocity * 0.075, Constants.Swerve.maxAngularVelocity * 0.075);
            if (thetaController.atSetpoint()) {
                swerve.xPosition(true);andThen(new InstantCommand(() ->
                swerve.drive(new Translation2d(0,0), 0, true, true)));
            } else {
                swerve.drive(new Translation2d(0,0), rotationVal, true, true);

            }
       
        
    }

    @Override
    public void end(boolean interuptted) {
        //swerve.xPosition(true);
    }



    
}
