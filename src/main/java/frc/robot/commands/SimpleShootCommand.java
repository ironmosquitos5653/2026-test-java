// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_drive = drive;
    addRequirements(m_ShooterSubsystem, m_IntakeSubsystem, m_drive);
    m_velocity = velocity;
    m_hoodPosition = hoodPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.stopWithX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setHood(m_hoodPosition);

    setShootSpeed(m_velocity);

    if (m_ShooterSubsystem.getVelocity() < -m_velocity * .90) {
      m_ShooterSubsystem.setAdvanceSpeed(1);
      jiggleIntake();
    }
  }

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
    return false;
  }

  public void setHood(double position) {
    m_ShooterSubsystem.setHoodPosition(position);
  }

  public void setShootSpeed(double velocity) {
    m_ShooterSubsystem.setShootSpeed(velocity);
  }

  private Timer intakeTimer;
  private boolean intakeIn;
  private boolean intakeOn = false;

  private void jiggleIntake() {

    if (!intakeOn) {
      if (intakeTimer == null) {
        intakeTimer = new Timer();
        intakeTimer.start();
      }

      if (intakeTimer.hasElapsed(1)) {
        intakeOn = true;
        intakeTimer.reset();
        intakeTimer.start();
      } else {
        return;
      }
    }

    if (intakeTimer == null) {
      intakeTimer = new Timer();
      intakeTimer.start();
    }
    m_IntakeSubsystem.setIntakeSpeed(-1);

    if (intakeIn && intakeTimer.hasElapsed(.5)) {
      intakeIn = false;
      intakeTimer.reset();
      intakeTimer.start();
    } else if (!intakeIn && intakeTimer.hasElapsed(.25)) {
      intakeIn = true;
      intakeTimer.reset();
      intakeTimer.start();
    }

    if (intakeIn) {
      m_IntakeSubsystem.setDeploySpeed(.4);
    } else {
      m_IntakeSubsystem.setDeploySpeed(-.4);
    }
  }
}
