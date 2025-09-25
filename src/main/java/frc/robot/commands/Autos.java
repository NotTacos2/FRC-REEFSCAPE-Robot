// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.commands.DriveCommand;
import frc.robot.subsystems.drive.commands.DriveToPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {

  public static Command mainAuto(RobotContainer bot) {
    return Commands.either(mainBlueAuto(bot), mainRedAuto(bot), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue);
  }

  private static Command mainBlueAuto(RobotContainer bot) {
    return Commands.sequence(new DriveToPoint(bot.m_drive, new Pose2d(0, 0.5, new Rotation2d(3 * Math.PI / 2))).withTimeout(5), bot.m_elevator.setGoal(3), new WaitCommand(2), new InstantCommand(() -> {
      bot.m_intake.setMotor(0.7);
    }), new WaitCommand(2), bot.m_elevator.setGoal(0), new InstantCommand(() -> {
      bot.m_intake.setMotor(0);
    }));
  }
  
  private static Command mainRedAuto(RobotContainer bot) {
    return Commands.sequence(new DriveToPoint(bot.m_drive, new Pose2d(0, 0.5, new Rotation2d(3 * Math.PI / 2))).withTimeout(5), bot.m_elevator.setGoal(3), new WaitCommand(2), new InstantCommand(() -> {
      bot.m_intake.setMotor(0.7);
    }), new WaitCommand(2), bot.m_elevator.setGoal(0), new InstantCommand(() -> {
      bot.m_intake.setMotor(0);
    }));
  }

  public static Command driveAuto(RobotContainer bot) {
    return new DriveCommand(bot.m_drive, new Pose2d(0, 0.7, new Rotation2d(0))).withTimeout(3);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
