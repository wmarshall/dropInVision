package frc.robot;

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

    public static interface Chassis {
        Rotation2d getYaw();

        Rotation2d getYawRate();

        void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    // meters, degrees
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    private static final double MAX_ROTATIONS_PER_SECOND = 2;

    private final StructPublisher<Pose2d> mt2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("PoseEstimate/MegaTag2", Pose2d.struct).publish();

    private Chassis chassis;

    public VisionPoseEstimator(Chassis chassis) {
        mt2Publisher.set(new Pose2d());
        this.chassis = chassis;

        LimelightHelpers.setCameraPose_RobotSpace("", ROBOT_TO_CAMERA.getX(), ROBOT_TO_CAMERA.getY(),
                ROBOT_TO_CAMERA.getZ(), ROBOT_TO_CAMERA.getRotation().getX(), ROBOT_TO_CAMERA.getRotation().getY(),
                ROBOT_TO_CAMERA.getRotation().getZ());
    }

    public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
        if (chassis.getYawRate().getRotations() > MAX_ROTATIONS_PER_SECOND) {
            return Optional.empty();
        }
        var est = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(""));
        return est.filter((pe) -> pe.tagCount > 0);
    }

    public void periodic() {
        LimelightHelpers.SetRobotOrientation("", chassis.getYaw().getDegrees(), 0, 0, 0, 0, 0);

        getPoseEstimate().ifPresent((pe) -> {
            mt2Publisher.set(pe.pose);
            // TODO: pull stddevs from network tables since LimelightHelpers doesn't expose
            // them
            chassis.addVisionMeasurement(pe.pose, pe.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
        });

    }

}
