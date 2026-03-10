// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeToggleCommad extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  boolean m_in;

  public IntakeToggleCommad(IntakeSubsystem intakeSubsystem, boolean in) {
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    m_in = in;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_in) m_IntakeSubsystem.setDeploySpeed(-.4);
    else m_IntakeSubsystem.setDeploySpeed(.4);
    m_IntakeSubsystem.setIntakeSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setDeploySpeed(0);
    m_IntakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
