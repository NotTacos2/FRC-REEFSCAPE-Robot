// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.drive.MecanumDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.PowerIntake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public final MecanumDrivetrain m_drive = new MecanumDrivetrain();
  public final Elevator m_elevator = new Elevator();
  public final Intake m_intake = new Intake();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_armController =
    new CommandXboxController(OperatorConstants.kArmControllerPort);

  public RobotContainer() {
    configureBindings();
    configureAutos();

    SmartDashboard.putNumberArray("Elevator Heights", AutoConstants.elevatorHeights);
  }

  private void configureBindings() {
    m_armController.y().onTrue(m_elevator.setGoal(3));
    m_armController.povUp().onTrue(m_elevator.up());
    m_armController.povDown().onTrue(m_elevator.down());
    m_armController.a().onTrue(m_elevator.setGoal(0));

    m_armController.x().onTrue(m_elevator.setPos(5));

    //TODO: Fix this so it doesn't conflict with other set power
    m_armController.rightBumper().onTrue(new PowerIntake(m_intake, 0.6).withTimeout(0.2));
  }

  private void configureAutos() {
    m_autoChooser.addOption("Main Auto", Autos.mainAuto(this));
    m_autoChooser.addOption("Drive Forward Auto", Autos.driveAuto(this));

    SmartDashboard.putData("Auto", m_autoChooser);
  }

  public Command getAuto() {
    return m_autoChooser.getSelected();
  }
  
  //TODO: Refactor
  public void teleop() {
    double drivePower = (1 - (m_elevator.m_elevMotor.getEncoder().getPosition() / AutoConstants.elevatorHeights[3])) * 0.5 + 0.5;
    
    if(m_driverController.rightBumper().getAsBoolean()) {
      drivePower = 0.3;
    }
    m_drive.driveCartesian(-m_driverController.getLeftY() * drivePower, m_driverController.getLeftX() * drivePower, m_driverController.getRightX() * drivePower);
    
    m_intake.setMotor((m_armController.getRightTriggerAxis() - m_armController.getLeftTriggerAxis()) * 0.7);
  }

  public void simulationDrive() {
    m_drive.driveCartesianFieldRelative(-m_driverController.getLeftY(), -m_driverController.getLeftX(), -m_driverController.getRightX());
  }

  public void reset() {
    m_elevator.reset();
    m_drive.reset();
  }
}
