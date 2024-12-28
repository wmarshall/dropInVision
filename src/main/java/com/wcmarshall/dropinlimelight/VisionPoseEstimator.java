package com.wcmarshall.dropinlimelight;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public final class VisionPoseEstimator {

    /**
     * Provides the methods needed to do first-class pose estimation
     */
    public static interface Chassis {

        Rotation2d getYaw();

        Rotation2d getYawPerSecond();

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

    // meters, radians. Robot origin to camera lens origin
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    // reject new poses if spinning too fast
    private static final double MAX_ROTATIONS_PER_SECOND = 2;

    private final StructPublisher<Pose2d> mt2Publisher;
    private final Chassis chassis;
    private final String limelightName, limelightHostname;

    /**
     * Create a VisionPoseEstimator
     *
     * @param chassis       the robot chassis to estimate the pose of
     * @param limelightName passed down to calls to LimelightHelpers, useful if you
     *                      have more than one Limelight on a robot
     */
    public VisionPoseEstimator(Chassis chassis, String limelightName) {

        this.chassis = chassis;
        this.limelightName = limelightName;
        this.limelightHostname = "limelight" + (limelightName != "" ? "-" + limelightName : "");

        mt2Publisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionPoseEstimator/" + this.limelightName, Pose2d.struct).publish();
        mt2Publisher.setDefault(new Pose2d());

        LimelightHelpers.setCameraPose_RobotSpace(limelightName, ROBOT_TO_CAMERA.getX(), ROBOT_TO_CAMERA.getY(),
                ROBOT_TO_CAMERA.getZ(), Math.toDegrees(ROBOT_TO_CAMERA.getRotation().getX()),
                Math.toDegrees(ROBOT_TO_CAMERA.getRotation().getY()),
                Math.toDegrees(ROBOT_TO_CAMERA.getRotation().getZ()));
    }

    /**
     * Create a VisionPoseEstimator
     *
     * @param chassis the robot chassis to estimate the pose of
     */
    public VisionPoseEstimator(Chassis chassis) {
        this(chassis, "");
    }

    /**
     * Get a pose estimate from the configured Limelight, if available
     *
     * @return An Optional containing MegaTag2 pose estimate from the Limelight, or
     *         Optional.empty if it is unavailable or untrustworthy
     */
    public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
        if (Math.abs(chassis.getYawPerSecond().getRotations()) > MAX_ROTATIONS_PER_SECOND) {
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
