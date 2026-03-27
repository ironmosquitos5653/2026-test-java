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
import frc.robot.util.Aimer;
import frc.robot.util.DistanceCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAndIntakeCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  DistanceCalculator m_DistanceCalculator;
  Aimer m_aimer;
  Drive m_drive;
  Timer timer = new Timer();

  private boolean m_timed;

  /** Creates a new ShootCommand. */
  public ShootAndIntakeCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      DistanceCalculator distanceCalculator,
      Aimer aimer,
      boolean timed) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_drive = drive;
    m_DistanceCalculator = distanceCalculator;
    m_aimer = aimer;
    m_timed = timed;
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("SHOOTING!!!");
    System.out.println("RainAndWalterAreAWESOME...veryAwesome");
    m_aimer.setTarget(m_aimer.getTargetPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] calculations = m_DistanceCalculator.getVelocityAndHood();
    double velocity = calculations[0];
    double hood = calculations[1];

    setHood(hood);
    setShootSpeed(velocity);

    if (m_ShooterSubsystem.getVelocity() < -velocity * .90) {
      m_ShooterSubsystem.setAdvanceSpeed(1);
    }
    if (timer.hasElapsed(1)) {
      m_IntakeSubsystem.setDeploySpeed(0);
    } else {
      m_IntakeSubsystem.setDeploySpeed(-.25);
    }

    m_IntakeSubsystem.setIntakeSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShootSpeed(0);
    m_ShooterSubsystem.setAdvanceSpeed(0);
    m_IntakeSubsystem.setDeploySpeed(0);
    m_IntakeSubsystem.setIntakeSpeed(0);
    m_ShooterSubsystem.setHoodPosition(HoodPosition.DOWN);
    m_aimer.setTarget(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // shoot while held.
    if (!m_timed) return false;

    return timer.hasElapsed(6);
  }

  public void setHood(double position) {
    m_ShooterSubsystem.setHoodPosition(position);
  }

  public void setShootSpeed(double velocity) {
    m_ShooterSubsystem.setShootSpeed(velocity);
  }
}
