package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AprilTags.Vision;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier lockOnTag;
    private PIDController thetaController;
    private double rotationVal, translationVal, strafeVal;
    private WeekZeroVision vision;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier lockOnTag, WeekZeroVision vision) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.vision = vision;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.lockOnTag = lockOnTag;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.01, 0, .01);
        thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        boolean aimAtTag = lockOnTag.getAsBoolean();
        translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
        strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
        if(aimAtTag) {
            vision.aimAtTarget();
            thetaController.setSetpoint(vision.aimAtTarget());
            rotationVal = thetaController.calculate(MathUtil.inputModulus(s_Swerve.getPose().getRotation().getDegrees(), -180, 180), rotationVal);
        } else if (!aimAtTag){
            rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband),3);
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
