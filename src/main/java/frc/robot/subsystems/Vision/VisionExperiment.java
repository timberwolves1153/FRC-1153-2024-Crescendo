package frc.robot.subsystems.Vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class VisionExperiment extends SubsystemBase{
    
    private PhotonCamera cam;
    private Swerve swerve;
    

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
;

    private PIDController tranlationPID;
    private PIDController rotationPID;
    private String fieldLayout;

    public VisionExperiment() {

        
        cam = new PhotonCamera("photonvision");
        swerve = new Swerve();

        tranlationPID = new PIDController(0, 0, 0);
        rotationPID = new PIDController(0, 0, 0);

        fieldLayout = AprilTagFields.k2024Crescendo.m_resourceFile;

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.


    
    }

    public void aimAtTarget() {
        var result = cam.getLatestResult();
        double targetRotation = result.getBestTarget().getYaw();

        rotationPID.calculate(targetRotation, 0);

        swerve.drive(new Translation2d(0,0), targetRotation, true, true);

        
    }

    public Pose3d estimateFieldRelativePose() {
        var result = cam.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, new Transform3d());
    }


    @Override
    public void periodic() {
        var result = cam.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        int targetID = target.getFiducialId();
        Transform3d pose3d = target.getBestCameraToTarget();
        // SmartDashboard.putNumber("target x", result.getBestTarget());
        // SmartDashboard.putNumber("target y", result.getBestTarget());
        SmartDashboard.putNumber("target yaw", result.getBestTarget().getYaw());
        SmartDashboard.putNumber("target ID", targetID);



        

    }
}
