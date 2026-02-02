// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  ClimbSubsystem m_climbSubsystem;
  double m_speed;

  public ClimbCommand(ClimbSubsystem climbSubsystem, boolean up) {
    m_climbSubsystem = climbSubsystem;
    m_speed = up ? 1.0 : -1.0;
  }

  public ClimbCommand(ClimbSubsystem climbSubsystem, double speed) {
    m_climbSubsystem = climbSubsystem;
    m_speed = speed;
  }

  public ClimbCommand(ClimbSubsystem m_ClimbSubsystem2) {
    //TODO Auto-generated constructor stub
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbSubsystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
