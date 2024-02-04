package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class VisionExperiment extends SubsystemBase implements AutoCloseable{
    public static class Hardware {
        AprilTagCamera[] cameras;

        public Hardware(AprilTagCamera... cameras) {
            this.cameras = cameras;
        }
    }
    private static VisionExperiment m_subsystem;
    private Supplier<Pose2d> m_poseSupplier;
    
    private PhotonCamera cam;
    private Swerve swerve;
    private AprilTagCamera[] aprilTagCameras;
    private Notifier m_cameraNotifier;
    private AtomicReference<List<EstimatedRobotPose>> m_estimatedRobotPose;
    private AtomicReference<List<Integer>> m_visibleTagIDs;
    
    private PhotonTrackedTarget target;


    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(9);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
   final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(21.7);
   private final double CAMERA_OFFSET = 23;


    private PhotonPoseEstimator photonPoseEstimator;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
;

    private PIDController tranlationPID;
    private PIDController rotationPID;
    private AprilTagFieldLayout fieldLayout;
    private Transform3d robotToCam;
    private static final String VISIBLE_TAGS_LOG_ENTRY = "/VisibleTags";
    private static final String ESTIMATED_POSES_LOG_ENTRY = "/EstimatedPoses";

    public VisionExperiment(Hardware visionHardware) {

        this.swerve = swerve;
        this.aprilTagCameras = visionHardware.cameras;
        this.m_estimatedRobotPose = new AtomicReference<List<EstimatedRobotPose>>();
        this.m_visibleTagIDs = new AtomicReference<List<Integer>>();

        cam = new PhotonCamera("intakeCam");
        robotToCam = new Transform3d(new Translation3d(0.33, 0.33, 0.2), new Rotation3d(0, CAMERA_PITCH_RADIANS, Math.toRadians(CAMERA_OFFSET))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        tranlationPID = new PIDController(0.1, 0, 0);
        rotationPID = new PIDController(0.08, 0, .01);
        rotationPID.enableContinuousInput(-180, 180);

        fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

       for(var camera : aprilTagCameras) camera.setPipelineIndex(0);

    
    }

    private static Hardware initializeHardware() {
        Hardware visionHardware = new Hardware(
            new AprilTagCamera(
                Constants.VisionSettings.INTAKECAM_NAME,
                Constants.VisionSettings.INTAKECAM_LOCATION,
                Constants.VisionSettings.INTAKECAM_RESOLUTION,
                Constants.VisionSettings.INTAKECAM_FOV)
        );
        return visionHardware;
    }
    // photonvision implimentation below
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPoses(Pose2d estimatedRobotPose) {
    //    photonPoseEstimator.setReferencePose(estimatedRobotPose);
    //    return photonPoseEstimator.update();
    //}

    public List<EstimatedRobotPose> getEstimatedRobotPoses() {
        return m_estimatedRobotPose.getAndSet(Collections.emptyList());
    }

    public List<Integer> getVisibleTagIDs() {
        return m_visibleTagIDs.get();
      }

    public void aimAtTarget() {
        var result = cam.getLatestResult();
        if (result.hasTargets()) {
        double targetRotation = result.getBestTarget().getYaw() - CAMERA_OFFSET;
        double pidval = rotationPID.calculate(targetRotation, 0);
        swerve.drive(new Translation2d(0,0), pidval, true, true);
        }
        
    }

    // public Pose3d estimateFieldRelativePose() {
    //     var result = cam.getLatestResult();
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, new Transform3d());
    // }

    public double calculateRange() {
        var result = cam.getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, 
                Units.inchesToMeters(57), 
                CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(result.getBestTarget().getPitch()));
            return range;
        }else {
            return 0;
        }
    }


    public void updateEstimatedGlobalPoses() {
        List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

        List<Integer> visibleTagIDs = new ArrayList<Integer>();
        HashSet<Pose3d> visibleTags = new HashSet<Pose3d>();
        List<Pose2d> loggedPoses = new ArrayList<Pose2d>();

        for(var camera : aprilTagCameras) {
            var result = camera.getLatestEstimatedPose();
            if (result == null) continue;
            result.targetsUsed.forEach((photonTrackedTarget) -> {
                if (photonTrackedTarget.getFiducialId() == -1) return;
                visibleTagIDs.add(photonTrackedTarget.getFiducialId());
                visibleTags.add(fieldLayout.getTagPose(photonTrackedTarget.getFiducialId()).get());
            });
            estimatedPoses.add(result);
            loggedPoses.add(result.estimatedPose.toPose2d());

            Logger.recordOutput(getName() + VISIBLE_TAGS_LOG_ENTRY, visibleTags.toArray(new Pose3d[0]));
            Logger.recordOutput(getName() + ESTIMATED_POSES_LOG_ENTRY, loggedPoses.toArray(new Pose2d[0]));

            m_visibleTagIDs.set(visibleTagIDs);
            m_estimatedRobotPose.set(estimatedPoses);

        }
    }

    public static VisionExperiment getInstance() {
        if (m_subsystem == null) {
            m_subsystem = new VisionExperiment(initializeHardware());
        }
        return m_subsystem;
    }

    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;
    }

    @Override
  public void close() {
    for (var camera : aprilTagCameras){ 
        camera.close(); 
    }
    m_cameraNotifier.close();
  }



    @Override
    public void periodic() {
        var result = cam.getLatestResult();
        photonPoseEstimator.update();
                //int targetID = target.getFiducialId();
        //Transform3d pose3d = target.getBestCameraToTarget();
        // SmartDashboard.putNumber("target x", result.getBestTarget());
        // SmartDashboard.putNumber("target y", result.getBestTarget());
        if (result.hasTargets()) {
            SmartDashboard.putNumber("target yaw", result.getBestTarget().getYaw());
            SmartDashboard.putNumber("target pitch radians", Units.degreesToRadians(result.getBestTarget().getPitch()));
            SmartDashboard.putNumber("range", calculateRange());
        }
       
       // SmartDashboard.putNumber("target ID", targetID);

        



        

    }
}
