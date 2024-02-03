package frc.robot.subsystems.Vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class VisionExperiment extends SubsystemBase{
    
    private PhotonCamera cam;
    private Swerve swerve;
    
    private PhotonTrackedTarget target;


    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(5);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
   final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(61.114);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
;

    private PIDController tranlationPID;
    private PIDController rotationPID;
    private String fieldLayout;

    public VisionExperiment(Swerve swerve) {

        
        cam = new PhotonCamera("intakeCam");
        this.swerve = swerve;
        

        tranlationPID = new PIDController(0, 0, 0);
        rotationPID = new PIDController(0, 0, 0);

        fieldLayout = AprilTagFields.k2024Crescendo.m_resourceFile;

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.


    
    }

    public void aimAtTarget() {
        var result = cam.getLatestResult();
        double targetRotation = result.getBestTarget().getYaw();

        rotationPID.calculate(targetRotation, 0);

        swerve.drive(new Translation2d(0,0), targetRotation, true, false);

        
    }

    // public Pose3d estimateFieldRelativePose() {
    //     var result = cam.getLatestResult();
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, new Transform3d());
    // }

    public double calculateRange() {
        var result = cam.getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(57.7), GOAL_RANGE_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            return range;
        }else {
            return 0;
        }
    }


    @Override
    public void periodic() {
        var result = cam.getLatestResult();
                //int targetID = target.getFiducialId();
        //Transform3d pose3d = target.getBestCameraToTarget();
        // SmartDashboard.putNumber("target x", result.getBestTarget());
        // SmartDashboard.putNumber("target y", result.getBestTarget());
        if (result.hasTargets()) {
            SmartDashboard.putNumber("target yaw", result.getBestTarget().getYaw());
            SmartDashboard.putNumber("range", calculateRange());
        }
       
       // SmartDashboard.putNumber("target ID", targetID);

        



        

    }
}
