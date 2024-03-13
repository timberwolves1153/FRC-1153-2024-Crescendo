package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class VelocityOffset extends Command{

    Swerve m_swerve;
    boolean m_isDone;
    BooleanSupplier isRunning;

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

    
    public VelocityOffset(Swerve swerve, BooleanSupplier isRunning) {
        this.m_swerve = swerve;
        this.isRunning = isRunning;
        shotTimer = new Timer();
        ranOnce = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //Starts shot timer after trigger press
        if (isRunning.getAsBoolean()) {
            if (!ranOnce) {
                shotTimer.start();
                ranOnce = true;
            }
        }
        //Get current translation of the drivetrain
        currentRobotTranslation = m_swerve.getPose().getTranslation(); 
        //Calculate angle relative to the speaker from current pose
        currentAngleToSpeaker = m_swerve.getSpeakerAngle(); 
        //Get current drivetrain velocities in field relative terms
        speeds = m_swerve.getFieldRelativeSpeeds(); 
        
        timeUntilShot = Constants.FieldConstants.TIME_UNTIL_SHOT - shotTimer.get();
        if (timeUntilShot < 0) {
            timeUntilShot = 0.00;
        }

        //Calculate change in x/y distance due to time and velocity
        moveDelta = new Translation2d(timeUntilShot*(speeds.vxMetersPerSecond),timeUntilShot*(speeds.vyMetersPerSecond));

        //futureRobotPose is the position the robot will be at timeUntilShot in the future
        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        //Angle to the speaker at future position
        futureAngleToSpeaker = m_swerve.getAngleToSpeaker(futureRobotTranslation);

        //The amount to add to the current angle to speaker to aim for the future
        correctedRotation = futureAngleToSpeaker;
        //correctedRotation = currentAngleToSpeaker; //Uncomment to disable future pose aiming
        // Get the future distance to speaker
        correctedDistance = m_swerve.getDistToSpeaker(futureRobotTranslation);
       // m_swerve.setVelocityOffset(correctedRotation,correctedDistance); //Pass the offsets to the drivetrain


              

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        shotTimer.reset();
        ranOnce = false;
        //m_isDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }

}