package frc.robot.subsystems.AprilTags;

    import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
    import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    
    public class WeekZeroVision extends SubsystemBase{
        
        public final PhotonCamera cam = new PhotonCamera("launcherCam");
        
        private PhotonTrackedTarget target;
    
    
        final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(23.3);
        final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
        final double GOAL_RANGE_METERS = Units.feetToMeters(3);
        final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-1.5);
        // Angle between horizontal and the camera.
       // final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(camResult.getBestTarget().getPitch());
        public final Transform3d cameraLocation = new Transform3d(new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(4.5), CAMERA_HEIGHT_METERS), new Rotation3d(0,CAMERA_PITCH_RADIANS, 0));
        //public final Transform3d cameraLocation = new Transform3d(new Translation3d(0 ,0, 0), new Rotation3d(0, 0, 0));
        // How far from the target we want to be
        
    
        //private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
        private PIDController tranlationPID;
        private PIDController rotationPID;
        private String fieldLayout;
    
        public WeekZeroVision() {

            
    
            tranlationPID = new PIDController(0, 0, 0);
            rotationPID = new PIDController(0, 0, 0);
    
            //fieldLayout = AprilTagFields.k2024Crescendo.m_resourceFile;
    
             //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    
    
        
        }
    
        public double aimAtTarget() {
            var result = cam.getLatestResult();
            if (result.hasTargets()) {
                double targetRotation = result.getBestTarget().getYaw();
                return targetRotation;
            } else {
                return 0;
            }
        }
    
        // public Pose3d estimateFieldRelativePose() {
        //     var result = cam.getLatestResult();
        //     PhotonTrackedTarget target = result.getBestTarget();
        //     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, new Transform3d());
        // }
    
        public double calculateRange() {
            PhotonPipelineResult result = cam.getLatestResult();
            PhotonTrackedTarget preferredTarget = getPreferredTarget(result);
            if (result.hasTargets()) {
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS, 
                    Units.inchesToMeters(57), 
                    CAMERA_PITCH_RADIANS, 
                    Units.degreesToRadians(preferredTarget.getPitch()));
                return range;
            }else {
                return 0;
            }
        }

        public PhotonTrackedTarget getPreferredTarget(PhotonPipelineResult result) {
             
            List<PhotonTrackedTarget> seenTags = result.getTargets();
            List<Integer> tagIds = result.getMultiTagResult().fiducialIDsUsed;
            //4 for Red (3 back up)
            // 7 for Blue (8 back up)
            boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);

            int preferredTag = 0;
            if (isBlue){
                if (tagIds.contains(Integer.valueOf(7))) {
                    preferredTag = 7;
                } else if (tagIds.contains(Integer.valueOf(8))) {
                    preferredTag = 8;
                } else {
                    preferredTag = 0;
                }
            } else {
                if (tagIds.contains(Integer.valueOf(4))) {
                    preferredTag = 4;
                } else if (tagIds.contains(Integer.valueOf(3))) {
                    preferredTag = 3;
                } else {
                    preferredTag = 0;
                }
            }

            if (preferredTag == 0) {
                return result.getBestTarget();
            }

            for (PhotonTrackedTarget tag : seenTags) {
                if (tag.getFiducialId() == preferredTag) {
                    return tag;
                }
            }

            return result.getBestTarget();
        }

        public boolean isOnTarget() {
            var result = cam.getLatestResult();
            if(result.hasTargets()) {
            if (Math.abs(result.getBestTarget().getYaw()) < 2) {
                return true;
            } else {
                return false;
            }         
        } else {
            return false;
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
                SmartDashboard.putNumber("calculated rotation", aimAtTarget());
                SmartDashboard.putBoolean("robot on target", isOnTarget());
                SmartDashboard.putNumber("target yaw", result.getBestTarget().getYaw());
                SmartDashboard.putNumber("range", calculateRange());
            }
        
            SmartDashboard.putNumber("vision timestamp secs", result.getTimestampSeconds());
            SmartDashboard.putBoolean("has targets", result.hasTargets());
            SmartDashboard.putBoolean("multitag pose", result.getMultiTagResult().estimatedPose.isPresent);
            //SmartDashboard.putNumber("seen tags", result.getMultiTagResult().fiducialIDsUsed.i);
    
        }
    }