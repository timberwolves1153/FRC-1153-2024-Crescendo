package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ObjectDetecting;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AprilTags.Vision;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier lockOnTag;
     private BooleanSupplier windhamAim;
    private PIDController thetaController;
    private double rotationVal, translationVal, strafeVal;
    private WeekZeroVision vision;
    private ObjectDetecting objectDetecting;
    
    private BooleanSupplier lockOnSpeaker;
    private PIDController translationController;

    Timer shotTimer;
    Boolean ranOnce;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    double currentAngleToSpeaker;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureAngleToSpeaker;

    ChassisSpeeds speeds;
    Translation2d moveDelta;

    /**The calculated the time until the note leaves based on the constant and time since button press */
    Double timeUntilShot; 
    DoubleSupplier m_trigger; 

    Double correctedDistance;
    Rotation2d correctedRotation;

    public TeleopSwerve(Swerve s_Swerve, 
        DoubleSupplier translationSup, 
        DoubleSupplier strafeSup, 
        DoubleSupplier rotationSup, 
        BooleanSupplier robotCentricSup, 
        BooleanSupplier lockOnTag,
        BooleanSupplier lockOnSpeaker,
        BooleanSupplier windhamAim, 
        WeekZeroVision vision) {
        
        this.s_Swerve = s_Swerve;
        this.vision = vision;
        shotTimer = new Timer();
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.lockOnTag = lockOnTag;
        this.windhamAim = windhamAim;
        this.lockOnSpeaker = lockOnSpeaker;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.015, 0.001, 0.0);
        thetaController.enableContinuousInput(-180, 180);
        translationController = new PIDController(0.05, 0, 0);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        boolean aimAtTag = lockOnTag.getAsBoolean();
        boolean aimAtSpeaker = lockOnSpeaker.getAsBoolean();
        
        if(aimAtTag) {
            translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
            strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
            thetaController.setSetpoint(0);
            rotationVal = thetaController.calculate(vision.aimAtTarget(), 0);
        }
           
        // } else if (lockOnObject) {
        //     //translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
        //     strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
        //     thetaController.setSetpoint(0);
        //     rotationVal = thetaController.calculate(objectDetecting.aimAtObject(), 0);
        //     translationController.setSetpoint(0);
        //     translationVal = translationController.calculate(objectDetecting.calculateRange(), 0);
        // } else if (!lockOnObject){
        //     translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
        //     strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
        //     rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband),3);
        
            else if(aimAtSpeaker) {
            
            boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);
            double directionFlip = isBlue ? 1.0 : -1.0;

            translationVal = Math.pow(directionFlip* MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
            strafeVal = Math.pow(directionFlip* MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);

            
            thetaController.setSetpoint(Math.toDegrees(s_Swerve.getSpeakerAngle()));
            rotationVal = thetaController.calculate(s_Swerve.getAngleAsDouble(), Math.toDegrees(s_Swerve.getSpeakerAngle()) + 180);
        } else if (windhamAim.getAsBoolean()) {
            shotTimer.start();
            translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
            strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
            currentRobotTranslation = s_Swerve.getPose().getTranslation(); 
            //Calculate angle relative to the speaker from current pose
            currentAngleToSpeaker = s_Swerve.getSpeakerAngle(); 
            //Get current drivetrain velocities in field relative terms
            speeds = s_Swerve.getFieldRelativeSpeeds(); 
            
            timeUntilShot = 4- shotTimer.get();
           // timeUntilShot = Constants.FieldConstants.TIME_UNTIL_SHOT - shotTimer.get();
            if (timeUntilShot < 0) {
                timeUntilShot = 0.00;
            }
    
            //Calculate change in x/y distance due to time and velocity
            moveDelta = new Translation2d(timeUntilShot*(speeds.vxMetersPerSecond),timeUntilShot*(speeds.vyMetersPerSecond));
    
            //futureRobotPose is the position the robot will be at timeUntilShot in the future
            futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
            //Angle to the speaker at future position
            futureAngleToSpeaker = s_Swerve.getAngleToSpeaker(futureRobotTranslation);
            thetaController.setSetpoint(futureAngleToSpeaker.getDegrees());
            rotationVal = thetaController.calculate(s_Swerve.getAngleAsDouble(), (futureAngleToSpeaker.getDegrees()+ 90+180));
            
        }
        else {
            translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
            strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
            rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband),3);

            shotTimer.stop();
            shotTimer.reset();
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
