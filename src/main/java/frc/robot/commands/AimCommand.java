// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TurnToPoseController;

public class AimCommand extends Command {

  Pose2d redTarget = new Pose2d(new Translation2d(11.9, 4.6), new Rotation2d(0));
  Pose2d blueTarget = new Pose2d(new Translation2d(4.6, 4.0), new Rotation2d(0));

  TurnToPoseController turnController = new TurnToPoseController(3, 0, .1);

  Drive m_Drive;

  public AimCommand(Drive drive) {
    m_Drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omega = m_Drive.getTurnToPoseOutput(getTargetPose2d(), turnController);
    // No translation, rotate in place using chassis speeds
    SmartDashboard.putNumber("Aim Speed", omega);
    m_Drive.runVelocity(new ChassisSpeeds(0.0, 0.0, -omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Pose2d getTargetPose2d() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redTarget;
    }
    return blueTarget;
  }
}
