// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Aimer;
import frc.robot.util.DistanceCalculator;
import frc.robot.util.TurnToPoseController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Drive m_drive;
  TurnToPoseController turnController = new TurnToPoseController(4, 0, 0);
  DistanceCalculator m_distanceCalculator;
  Aimer m_aimer;
  private boolean m_timed;

  Timer timer = new Timer();
  private Timer intakeTimer;
  private boolean intakeIn;
  private boolean intakeOn = false;
  private double m_shootTime = 6;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      DistanceCalculator distanceCalculator,
      Aimer aimer,
      double shootTime) {
    this(shooterSubsystem, intakeSubsystem, drive, distanceCalculator, aimer, true);
    m_shootTime = shootTime;
  }
  /** Creates a new ShootCommand. */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      DistanceCalculator distanceCalculator,
      Aimer aimer,
      boolean timed) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_distanceCalculator = distanceCalculator;
    m_aimer = aimer;
    m_drive = drive;
    m_timed = timed;
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer = null;
    intakeOn = false;
    intakeIn = true;
    timer.reset();
    timer.start();
    m_aimer.setTarget(m_distanceCalculator.getTargetPose2d());
    System.out.println("SHOOTING!!!");
    System.out.println("RainAndWalterAreAWESOME");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // aim();
    double[] calculations = m_distanceCalculator.getVelocityAndHood();
    double velocity = calculations[0];
    double hood = calculations[1];

    setHood(hood);
    setShootSpeed(velocity);

    if (m_ShooterSubsystem.getVelocity() < -velocity * .90) {
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
    m_aimer.setTarget(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // shoot while held.
    if (!m_timed) return false;

    return timer.hasElapsed(m_shootTime);
  }

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
