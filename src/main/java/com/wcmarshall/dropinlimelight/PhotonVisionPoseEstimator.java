package com.wcmarshall.dropinlimelight;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public class PhotonVisionPoseEstimator {
    private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final PhotonPoseEstimator poseEstimator;
    private final PhotonCamera camera;
    // reject new poses if spinning too fast
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);

    private final StructPublisher<Pose2d> posePublisher;
    private final Chassis chassis;
    private final String cameraName;

    /**
     * Create a VisionPoseEstimator
     *
     * @param cameraName    passed down to calls to LimelightHelpers, useful if you
     *                      have more than one Limelight on a robot
     * @param chassis       the robot chassis to estimate the pose of
     * @param robotToCamera Transform from robot origin to camera lens origin
     */
    public PhotonVisionPoseEstimator(String cameraName, Chassis chassis, Transform3d robotToCamera) {

        this.chassis = chassis;
        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera);

        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionPoseEstimator/photon/" + this.cameraName, Pose2d.struct).publish();
        posePublisher.setDefault(new Pose2d());
    }

    /**
     * Create a VisionPoseEstimator
     *
     * @param chassis       the robot chassis to estimate the pose of
     * @param robotToCamera Transform from robot origin to camera lens origin
     */
    public PhotonVisionPoseEstimator(Chassis chassis, Transform3d robotToCamera) {
        this("camera", chassis, robotToCamera);
    }

    public Optional<EstimatedRobotPose> getPoseEstimate() {
        if (chassis.getYawPerSecond().abs(RotationsPerSecond) > MAX_ANGULAR_VELOCITY.abs(RotationsPerSecond)) {
            return Optional.empty();
        }
        Optional<EstimatedRobotPose> est = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            // TODO: filter changes?
            est = poseEstimator.update(change);
        }

        return est.filter((pe) -> pe.targetsUsed.size() > 0);
    }

    /**
     * Update the limelight's robot orientation
     */
    public void periodic() {
        // TODO: account for laggy yaw?
        this.poseEstimator.addHeadingData(Timer.getTimestamp(), chassis.getYaw());

        getPoseEstimate().ifPresent((pe) -> {
            var pose2d = pe.estimatedPose.toPose2d();
            posePublisher.set(pose2d);
            pe.targetsUsed.get(0).getPoseAmbiguity();
            chassis.addVisionMeasurement(pose2d, pe.timestampSeconds,
                    // TODO: dynamic stddev
                    VecBuilder.fill(0.9, 0.9, Double.POSITIVE_INFINITY));
        });

    }
}
