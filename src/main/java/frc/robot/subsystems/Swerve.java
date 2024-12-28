package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.wcmarshall.dropinlimelight.VisionPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase implements VisionPoseEstimator.Chassis {

    private static final double TRACK_WIDTH = Units.inchesToMeters(24), TRACK_LENGTH = Units.inchesToMeters(24);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2),
            new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
            new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
            new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2));

    private final StructPublisher<Pose2d> odometryPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("PoseEstimate/Odometry", Pose2d.struct).publish();

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(),
            getModulePositions().toArray(new SwerveModulePosition[0]),
            new Pose2d());

    private final Pigeon2 imu = new Pigeon2(0);

    public Swerve() {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    }

    public Rotation2d getYaw() {
        return imu.getRotation2d();
    }

    public Rotation2d getYawPerSecond() {
        return Rotation2d.fromDegrees(imu.getRate());
    }

    public List<SwerveModulePosition> getModulePositions() {
        // FIXME: Left as an exercise for the reader
        return List.of(new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
                new SwerveModulePosition());
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions().toArray(new SwerveModulePosition[0]));
        odometryPublisher.set(poseEstimator.getEstimatedPosition());
    }

}
