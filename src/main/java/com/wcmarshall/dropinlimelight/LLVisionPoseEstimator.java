package com.wcmarshall.dropinlimelight;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;

public final class LLVisionPoseEstimator {

    /**
     * Provides the methods needed to do first-class pose estimation
     */
    public static interface Chassis {

        Rotation2d getYaw();

        AngularVelocity getYawPerSecond();

        /**
         * Passthrough to {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
         * addVisionMeasurement
         *
         * @param pose
         * @param timestampSeconds
         * @param visionMeasurementStdDevs
         */
        void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    // reject new poses if spinning too fast
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);

    private final StructPublisher<Pose2d> mt2Publisher;
    private final Chassis chassis;
    private final String limelightName, limelightHostname;

    /**
     * Create a VisionPoseEstimator
     *
     * @param limelightName passed down to calls to LimelightHelpers, useful if you
     *                      have more than one Limelight on a robot
     * @param chassis       the robot chassis to estimate the pose of
     * @param robotToCamera Transform from robot origin to camera lens origin
     */
    public LLVisionPoseEstimator(String limelightName, Chassis chassis, Transform3d robotToCamera) {

        this.chassis = chassis;
        this.limelightName = limelightName;
        this.limelightHostname = "limelight" + (limelightName != "" ? "-" + limelightName : "");

        mt2Publisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionPoseEstimator/limelight/" + this.limelightName, Pose2d.struct).publish();
        mt2Publisher.setDefault(new Pose2d());

        LimelightHelpers.setCameraPose_RobotSpace(limelightName, robotToCamera.getX(), robotToCamera.getY(),
                robotToCamera.getZ(), Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ()));
    }

    /**
     * Create a VisionPoseEstimator
     *
     * @param chassis       the robot chassis to estimate the pose of
     * @param robotToCamera Transform from robot origin to camera lens origin
     */
    public LLVisionPoseEstimator(Chassis chassis, Transform3d robotToCamera) {
        this("", chassis, robotToCamera);
    }

    /**
     * Get a pose estimate from the configured Limelight, if available
     *
     * @return An Optional containing MegaTag2 pose estimate from the Limelight, or
     *         Optional.empty if it is unavailable or untrustworthy
     */
    public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
        if (chassis.getYawPerSecond().abs(RotationsPerSecond) > MAX_ANGULAR_VELOCITY.abs(RotationsPerSecond)) {
            return Optional.empty();
        }
        var est = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName));
        return est.filter((pe) -> pe.tagCount > 0);
    }

    /**
     * Update the limelight's robot orientation
     */
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(limelightName, chassis.getYaw().getDegrees(), 0, 0, 0, 0, 0);

        getPoseEstimate().ifPresent((pe) -> {
            mt2Publisher.set(pe.pose);
            // LimelightHelpers doesn't expose a helper method for these, layout is:
            // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
            // MT2pitch, MT2yaw]
            var stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightHostname, "stddevs");
            chassis.addVisionMeasurement(pe.pose, pe.timestampSeconds,
                    VecBuilder.fill(stddevs[6], stddevs[7], Double.POSITIVE_INFINITY));
        });

    }

}
