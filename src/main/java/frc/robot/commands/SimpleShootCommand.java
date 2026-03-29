// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPosition;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleShootCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Drive m_drive;

  double m_velocity;
  double m_hoodPosition;

  public SimpleShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      double velocity,
      double hoodPosition) {

    // addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShootSpeed(0);
    m_ShooterSubsystem.setAdvanceSpeed(0);
    m_IntakeSubsystem.setDeploySpeed(0);
    m_IntakeSubsystem.setIntakeSpeed(0);
    m_ShooterSubsystem.setHoodPosition(HoodPosition.DOWN);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
