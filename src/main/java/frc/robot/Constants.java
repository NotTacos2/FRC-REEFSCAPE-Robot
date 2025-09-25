// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public final class Constants {

  public static final double dt = 0.02;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
  }

  public static class HardwareConstants {
    public static final int kFrontLeftMotorID = 5;
    public static final int kRearLeftMotorID = 4;
    public static final int kFrontRightMotorID = 3;
    public static final int kRearRightMotorID = 2;

    public static final int kElevatorMotorID = 6;
    public static final int kIntakeMotorID = 7;

    public static final double kXSpeed = 5;
    public static final double kYSpeed = 3;
    public static final double kRotSpeed = Math.PI;

    public static final String kLimelightName = "limelight-crash";
  }

  public static class AutoConstants {
    public static final Pose2d redStart = new Pose2d(10, 4, new Rotation2d(0));
    public static final Pose2d blueStart = new Pose2d(7.5, 4, new Rotation2d(Math.PI));
    
    //These are elevator heights 1-4
    //Not a constant...
    public static double[] elevatorHeights = {0, 17, 34, 58};
  }
}
