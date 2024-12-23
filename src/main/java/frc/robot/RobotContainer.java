// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final VisionPoseEstimator vpe = new VisionPoseEstimator(swerve);

  public RobotContainer() {
    // addPeriodic, but accessible from down here
    Commands.run(vpe::periodic);
  }

  public Command getAutonomousCommand() {
    return Commands.print("Autonomous Command");
  }
}
