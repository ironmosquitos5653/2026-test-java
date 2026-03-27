// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DistanceCalculator;
import frc.robot.util.TurnToPoseController;

public class FirstShootCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  Drive m_drive;
  DistanceCalculator m_distanceCalculator;
  Timer timer = new Timer();

  /** Creates a new ShootCommand. */
  public FirstShootCommand(
      ShooterSubsystem shooterSubsystem, Drive drive, DistanceCalculator distanceCalculator) {
    m_ShooterSubsystem = shooterSubsystem;
    m_drive = drive;
    m_distanceCalculator = distanceCalculator;
    addRequirements(m_ShooterSubsystem, m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aim();

    double[] calculations = m_distanceCalculator.getVelocityAndHood();
    double velocity = calculations[0];
    double hood = calculations[1];

    setHood(hood);
    setShootSpeed(velocity);

    if (m_ShooterSubsystem.getVelocity() < -velocity * .90) {
      m_ShooterSubsystem.setAdvanceSpeed(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShootSpeed(0);
    m_ShooterSubsystem.setAdvanceSpeed(0);
    m_ShooterSubsystem.setHoodPosition(HoodPosition.DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }

  TurnToPoseController turnController = new TurnToPoseController(1, 0, 0);

  public void aim() {
    double omega =
        m_drive.getTurnToPoseOutput(m_distanceCalculator.getTargetPose2d(), turnController);
    m_drive.runVelocity(new ChassisSpeeds(0.0, 0.0, -omega));
  }

  public void setHood(double position) {
    m_ShooterSubsystem.setHoodPosition(position);
  }

  public void setShootSpeed(double velocity) {
    m_ShooterSubsystem.setShootSpeed(velocity);
  }
}
