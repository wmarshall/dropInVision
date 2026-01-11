// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.wcmarshall.dropinlimelight.LLVisionPoseEstimator;
import com.wcmarshall.dropinlimelight.PhotonVisionPoseEstimator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final LLVisionPoseEstimator limelight0 = new LLVisionPoseEstimator(swerve, Transform3d.kZero);
  private final LLVisionPoseEstimator limelight1 = new LLVisionPoseEstimator("limelight1", swerve, new Transform3d(
      new Translation3d(Inches.of(12), Inches.of(14), Inches.of(6)),
      new Rotation3d(Degrees.of(0), Degrees.of(12), Degrees.of(30))));
  private final PhotonVisionPoseEstimator photon0 = new PhotonVisionPoseEstimator(swerve, Transform3d.kZero);

  public RobotContainer() {
    // addPeriodic, but accessible from down here
    Commands.run(limelight0::periodic).withName("LL0::periodic");
    Commands.run(limelight1::periodic).withName("LL1::periodic");
    Commands.run(photon0::periodic).withName("Photon0::periodic");
  }

  public Command getAutonomousCommand() {
    return Commands.print("Autonomous Command");
  }
}
