// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class DashboardSubsystem extends SubsystemBase {
  private int iterations = 1;

  private double frontLeftDrive = 0.0;
  private double frontRightDrive = 0.0;
  private double backLeftDrive = 0.0;
  private double backRightDrive = 0.0;

  private double shooter1 = 0.0;
  private double shooter2 = 0.0;
  private double shooter3 = 0.0;

  private Drive m_drive;
  private ShooterSubsystem m_shooter;

  public DashboardSubsystem(Drive drive, ShooterSubsystem shooter) {
    m_drive = drive;
    m_shooter = shooter;

    iterations++;

    ShuffleboardTab tab = Shuffleboard.getTab("motors");
    tab.addNumber("frontLeftDrive", this::getDriveMotor0).withPosition(0, 0).withSize(4, 0);
    tab.addNumber("frontRightDrive", this::getDriveMotor1).withPosition(2, 0).withSize(4, 0);
    tab.addNumber("backLeftDrive", this::getDriveMotor2).withPosition(4, 0).withSize(4, 0);
    tab.addNumber("backRightDrive", this::getDriveMotor3).withPosition(6, 0).withSize(4, 0);
    tab.addNumber("shooter1", this::getShootMotor1).withPosition(0, 2).withSize(4, 0);
    tab.addNumber("shooter2", this::getShootMotor2).withPosition(2, 2).withSize(4, 0);
    tab.addNumber("shooter3", this::getShootMotor3).withPosition(4, 2).withSize(4, 0);
  }

  private double getDriveMotor0() {
    return frontLeftDrive / iterations;
  }

  private double getDriveMotor1() {
    return frontRightDrive / iterations;
  }

  private double getDriveMotor2() {
    return backLeftDrive / iterations;
  }

  private double getDriveMotor3() {
    return backRightDrive / iterations;
  }

  private double getShootMotor1() {
    return shooter1 / iterations;
  }

  private double getShootMotor2() {
    return shooter2 / iterations;
  }

  private double getShootMotor3() {
    return shooter3 / iterations;
  }

  @Override
  public void periodic() {
    frontLeftDrive += m_drive.getDriveVelocity(0);
    frontRightDrive += m_drive.getDriveVelocity(1);
    backLeftDrive += m_drive.getDriveVelocity(2);
    backRightDrive += m_drive.getDriveVelocity(3);
    shooter1 += m_shooter.getShooterVelocity(0);
    shooter2 += m_shooter.getShooterVelocity(1);
    shooter3 += m_shooter.getShooterVelocity(2);
  }
}
