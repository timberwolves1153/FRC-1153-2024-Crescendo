package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetecting extends SubsystemBase {

    private PhotonCamera obCam;

    private PhotonTrackedTarget target;

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(23.3);
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(0);

    final double GOAL_RANGE_METERS = Units.inchesToMeters(0);
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(10); // NEEDS TO BE ADJUSTED

    private PIDController tranlationPID;
    private PIDController rotationPID;

    public ObjectDetecting() {
        obCam = new PhotonCamera("ObjectDetectionCamera");

        tranlationPID = new PIDController(0, 0, 0);
        rotationPID = new PIDController(0, 0, 0);

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
                                                                                                            
    }

    public double aimAtObject() {
        var result = obCam.getLatestResult();
        if (result.hasTargets()) {
        double targetRotation = result.getBestTarget().getYaw();
        return targetRotation;
        }else {
        return 0;
        }
    }

    public double calculateRange() {
            var result = obCam.getLatestResult();
            if (result.hasTargets()) {
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS, 
                    Units.inchesToMeters(0), 
                    CAMERA_PITCH_RADIANS, 
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
                return range;
            }else {
                return 0;
            }
        }
    
    
        @Override
        public void periodic() {
            var result = obCam.getLatestResult();
            //int targetID = target.getFiducialId();
            //Transform3d pose3d = target.getBestCameraToTarget();
            // SmartDashboard.putNumber("target x", result.getBestTarget());
            // SmartDashboard.putNumber("target y", result.getBestTarget());
            if (result.hasTargets()) {
                SmartDashboard.putNumber("object target yaw", result.getBestTarget().getYaw());
                SmartDashboard.putNumber("object range", calculateRange());
            }
           
            // SmartDashboard.putNumber("target ID", targetID);
    
        }
    }