// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.testbots.SwerveAutoSimRobot;

public final class Main {
  private Main() {}

  public static Rotation2d plan(double start, double finish) {
    return Rotation2d.fromDegrees(finish).minus(Rotation2d.fromDegrees(start));
  }

  public static void main(String... args) {
    // RobotBase.startRobot(ElevatorSimRobot::new);
     RobotBase.startRobot(SwerveAutoSimRobot::new);
    //  RobotBase.startRobot(AutoSimRobot::new);
  }
}
