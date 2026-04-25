// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double delay = 0;

  Timer timer = new Timer();
  Timer stuckTimer = null;
  private Timer intakeTimer;
  private boolean intakeIn;
  private boolean intakeOn = false;
  private double m_shootTime = 6;
  boolean shooting = false;

  // Progressing is Start -> Delayed -> Watching -> Reversing -> Delayed ...
  private enum StuckState {
    START, // Starting up, indexer not enabled.
    DELAYED, // Delayed because indexer just started.
    WATCHING, // Watching for stuck condition.
    REVERSING // Reversing because stuck condition met.
  }

  private StuckState stuckState = StuckState.START;

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
    // addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer = null;
    stuckTimer = null;
    intakeOn = false;
    intakeIn = true;
    shooting = false;
    timer.reset();
    timer.start();
    m_aimer.setTarget(m_aimer.getTargetPose());
    System.out.println("SHOOTING!!!");
    System.out.println("RainAndWalterAreAWESOME");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] calculations = m_distanceCalculator.getVelocityAndHood();

    // distance = 3.9
    // rpm = 3300 hood = .08

    // distance = 2.09
    // rpm = 3100 hood = .05

    // distance= 5
    // rpm = 3855 hood = .085

    double velocity = calculations[0];
    double hood = calculations[1];
    SmartDashboard.putNumber("The hood", hood);
    if (m_timed) {
      aim();
    }

    setHood(hood);
    setShootSpeed(velocity);

    // jiggleIntake();

    if (shooting || m_ShooterSubsystem.getVelocity() < -velocity * 1) {
      if (delay == 0) {
        delay = timer.get();
      } else if (delay != 0 && timer.hasElapsed(0 + delay)) {

        // Out advance motor speed.
        m_ShooterSubsystem.setOuterAdvanceSpeed(.75);
        m_ShooterSubsystem.setAdvanceSpeed(1);
        if (!shooting) {
          SmartDashboard.putNumber("Startup", timer.get());
        }
        shooting = true;
        // time to wait to put intake in
        if (timer.hasElapsed(2.5) && !timer.hasElapsed(6)) {
          // Intake deploy motor speed
          m_IntakeSubsystem.setDeploySpeed(.4);
          m_IntakeSubsystem.setIntakeSpeed(-1);
        } else {
          m_IntakeSubsystem.setDeploySpeed(0);
          m_IntakeSubsystem.setIntakeSpeed(0);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShootSpeed(0);
    m_ShooterSubsystem.setAdvanceSpeed(0);
    m_ShooterSubsystem.setOuterAdvanceSpeed(0);
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

    m_drive.runVelocity(new ChassisSpeeds(0.0, 0.0, -m_aimer.calculate()));
  }

  public void setHood(double position) {
    m_ShooterSubsystem.setHoodPosition(position);
  }

  public void setShootSpeed(double velocity) {
    SmartDashboard.putNumber("velocity", velocity);
    m_ShooterSubsystem.setShootSpeed(velocity);
  }

  private boolean stuck() {
    if (stuckTimer == null) {
      stuckTimer = new Timer();
      stuckTimer.start();
    }

    // Delayed State prevents us from reporting stuck when indexer is ramping up.
    if (stuckState == StuckState.DELAYED) {
      if (stuckTimer.hasElapsed(.5)) {
        stuckState = StuckState.WATCHING;
        stuckTimer.reset();
        stuckTimer.start();
      } else {
        return false;
      }
    }

    if (stuckState == StuckState.WATCHING && m_ShooterSubsystem.indexStuck()) {
      stuckState = StuckState.REVERSING;
      stuckTimer.reset();
      stuckTimer.start();
    }

    if (stuckState == StuckState.REVERSING) {
      if (stuckTimer.hasElapsed(.5)) {
        stuckState = StuckState.DELAYED;
        stuckTimer.reset();
        stuckTimer.start();
        return false;
      }
      m_ShooterSubsystem.setAdvanceSpeed(-1);
      return true;
    }
    return false;
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
