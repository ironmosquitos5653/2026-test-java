// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  double m_speed1;
  double m_speed2;
  Timer timer = new Timer();

  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, double speed1, double speed2) {
    m_ShooterSubsystem = shooterSubsystem;
    m_speed1 = speed1;
    m_speed2 = speed2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("SHOOTING!!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.setSpeed1(m_speed1);
    m_ShooterSubsystem.setSpeed2(m_speed2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setSpeed1(0);
    m_ShooterSubsystem.setSpeed2(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(10);
  }
}
