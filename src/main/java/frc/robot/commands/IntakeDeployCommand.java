// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeDeployCommand extends Command {

  IntakeSubsystem m_IntakeSubsystem;
  boolean m_in;
  ShooterSubsystem m_ShooterSubsystem;

  public IntakeDeployCommand(
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, boolean in) {
    m_IntakeSubsystem = intakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_in = in;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_in) {
      // m_IntakeSubsystem.setDeploySpeed(-.5);
      m_ShooterSubsystem.setHoodSpeed(-.5);
    } else {
      // m_IntakeSubsystem.setDeploySpeed(.5);
      m_ShooterSubsystem.setHoodSpeed(.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setDeploySpeed(0);
    m_ShooterSubsystem.setHoodSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
