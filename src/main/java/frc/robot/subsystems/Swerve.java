package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {

    private DoubleEntry simAngle = NetworkTableInstance.getDefault().getDoubleTopic("sim_angle").getEntry(90);
    private StructPublisher<Pose2d> mt1 = NetworkTableInstance.getDefault()
            .getStructTopic("MegaTag1", Pose2d.struct).publish();
    private StructPublisher<Pose2d> mt2 = NetworkTableInstance.getDefault()
            .getStructTopic("MegaTag2", Pose2d.struct).publish();

    public Swerve() {
        simAngle.set(90);
        mt1.set(new Pose2d());
        mt2.set(new Pose2d());
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(simAngle.get());
    }

    @Override
    public void periodic() {
        super.periodic();
        LimelightHelpers.SetRobotOrientation("", getYaw().getDegrees(), 0, 0, 0, 0, 0);

        var mt1Measurement = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue(""));
        mt1Measurement.ifPresent((pe) -> {
            mt1.set(pe.pose);
        });
        var mt2Measurement = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(""));
        mt2Measurement.ifPresent((pe) -> {
            mt2.set(pe.pose);
        });
    }
}
