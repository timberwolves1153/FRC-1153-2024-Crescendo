package frc.robot.subsystems.AprilTags;

import java.io.Closeable;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class AprilTagCamera implements Runnable, Closeable {
    private final double APRILTAG_POSE_AMBIGUITY_THRESHOLD = 0.2;

    public enum Resolution {
        RES_320_260(320, 240),
        RES_640_480(640, 480),
        RES_800_600(800, 600),
        RES_1280_720(1280, 720);

        public final int width;
        public final int height;

        private Resolution(int width, int height) {
            this.width = width;
            this.height = height;
        }
    }
    private PhotonCamera aprilTagCamera;
    private PhotonPoseEstimator poseEstimator;
    private Transform3d locationOnBot;
    private PhotonCameraSim cameraSim;
    private AtomicReference<EstimatedRobotPose> atomicEstimateRobotPose;
    
    public AprilTagCamera(String name, Transform3d locationOnBot, Resolution resolution, Rotation2d fovDiag ) {

        this.aprilTagCamera = new PhotonCamera(name);
        this.locationOnBot = locationOnBot;
        var fieldlayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        fieldlayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        this.poseEstimator = new PhotonPoseEstimator(
                fieldlayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                aprilTagCamera, 
                locationOnBot);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        this.atomicEstimateRobotPose = new AtomicReference<EstimatedRobotPose>();

        var cameraProperties = SimCameraProperties.PERFECT_90DEG();
        cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
        this.cameraSim = new PhotonCameraSim(aprilTagCamera, cameraProperties);
    }

    @Override
    public void run() {
        // retunr if camera or field layout don't load
        if(poseEstimator == null || aprilTagCamera == null) return;

        // upadtes and logs inputs
        PhotonPipelineResult pipelineResult = aprilTagCamera.getLatestResult();

        // reutrn if result is invalid or does not exist
        if (!pipelineResult.hasTargets()) return;

        if (pipelineResult.targets.size() == 1 
            && pipelineResult.targets.get(0).getPoseAmbiguity() > APRILTAG_POSE_AMBIGUITY_THRESHOLD) return;

        // update pose estimate
        poseEstimator.update(pipelineResult).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;

            //check if measurement in on the field
            if(estimatedPose.getX() > 0.0 && estimatedPose.getX() <= Constants.Field.FIELD_LENGTH
             && estimatedPose.getY() > 0 && estimatedPose.getY() <= Constants.Field.FIELD_WIDTH) {
                atomicEstimateRobotPose.set(estimatedRobotPose);
             }
        });
    }

    public EstimatedRobotPose getLatestEstimatedPose() {
        return atomicEstimateRobotPose.getAndSet(null);
    }

    public void setPipelineIndex(int index) {
        aprilTagCamera.setPipelineIndex(index);
    }

    public Transform3d gettTransform3d() {
        return locationOnBot;
    }

    @Override
    public void close() {
        aprilTagCamera.close();
    }
}