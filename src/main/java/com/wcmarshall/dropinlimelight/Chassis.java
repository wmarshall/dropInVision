package com.wcmarshall.dropinlimelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Provides the methods needed to do first-class pose estimation
 */
public interface Chassis {

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