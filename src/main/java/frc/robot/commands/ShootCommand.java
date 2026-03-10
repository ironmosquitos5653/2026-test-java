// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ShootData;
import frc.robot.util.TurnToPoseController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Drive m_drive;
  TurnToPoseController turnController = new TurnToPoseController(1, 0, 0);
  Timer timer = new Timer();

  public static Pose2d redTarget = new Pose2d(new Translation2d(11.9, 4.6), new Rotation2d(0));
  public static Pose2d blueTarget = new Pose2d(new Translation2d(4.6, 4.0), new Rotation2d(0));

  private boolean m_timed;

  private Timer intakeTimer;
  private boolean intakeIn;
  private boolean intakeOn = false;

  /** Creates a new ShootCommand. */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      boolean timed) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_drive = drive;
    m_timed = timed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer = null;
    intakeOn = false;
    intakeIn = true;
    timer.reset();
    timer.start();
    System.out.println("SHOOTING!!!");
    System.out.println("RainAndWalterAreAWESOME");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aim();

    double[] calculations = getVelocityAndHood(m_drive);
    double velocity = calculations[0];
    double hood = calculations[1];
    
    setHood(hood);
    setShootSpeed(velocity);

    if (m_ShooterSubsystem.getVelocity() < -velocity * .90) {
      m_ShooterSubsystem.setAdvanceSpeed(1);
      jiggleIntake();
      // intakeIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShootSpeed(0);
    m_ShooterSubsystem.setAdvanceSpeed(0);
    m_IntakeSubsystem.setDeploySpeed(0);
    m_IntakeSubsystem.setIntakeSpeed(0);
    setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // shoot while held.
    if (!m_timed) return false;

    return timer.hasElapsed(10);
  }

  public double geDistance() {
    return getDistance(m_drive);
  }

  public static double getDistance(Drive drive) {
    return drive.getPose().getTranslation().getDistance(getTargetPose2d().getTranslation());
  }

  public static Pose2d getTargetPose2d() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redTarget;
    }
    return blueTarget;
  }

  public void aim() {
    double omega = m_drive.getTurnToPoseOutput(getTargetPose2d(), turnController);
    m_drive.runVelocity(new ChassisSpeeds(0.0, 0.0, -omega));
  }

  public void setHood(double position) {
    m_ShooterSubsystem.setHoodPosition(position);
    // up .6
    // down .44
  }

  private double calculateShootSpeed(double distance) {
    return 3400;
  }

  public void setShootSpeed(double velocity) {
    m_ShooterSubsystem.setShootSpeed(velocity);
  }

  private void intakeIn() {

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

    m_IntakeSubsystem.setIntakeSpeed(-1);
    m_IntakeSubsystem.setDeploySpeed(.15);
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

  public static double[] getVelocityAndHood(Drive drive) {
    double distance = getDistance(drive);
    return getVelocityAndHood(distance);
  }

  public static double[] getVelocityAndHood(double distance) {
    // Use shootDatas to interpolate (or extrapolate) velocity and hood
    ShootData[] data = shootDatas;

    // If distance is below the first entry, extrapolate using first two
    if (distance <= data[0].distance()) {
      ShootData a = data[0];
      ShootData b = data[1];
      double t = (distance - a.distance()) / (b.distance() - a.distance());
      if (Double.isNaN(t) || Double.isInfinite(t)) t = 0.0;
      double vel = a.velocity() + t * (b.velocity() - a.velocity());
      double hood = a.hood() + t * (b.hood() - a.hood());
      return new double[] {vel, hood};
    }

    // If distance is above the last entry, extrapolate using last two
    int last = data.length - 1;
    if (distance >= data[last].distance()) {
      ShootData a = data[last - 1];
      ShootData b = data[last];
      double t = (distance - a.distance()) / (b.distance() - a.distance());
      if (Double.isNaN(t) || Double.isInfinite(t)) t = 0.0;
      double vel = a.velocity() + t * (b.velocity() - a.velocity());
      double hood = a.hood() + t * (b.hood() - a.hood());
      return new double[] {vel, hood};
    }

    // Otherwise find the interval [i, i+1] that contains distance
    for (int i = 0; i < data.length - 1; i++) {
      ShootData a = data[i];
      ShootData b = data[i + 1];
      if (distance >= a.distance() && distance <= b.distance()) {
        double t = (distance - a.distance()) / (b.distance() - a.distance());
        if (Double.isNaN(t) || Double.isInfinite(t)) t = 0.0;
        double vel = a.velocity() + t * (b.velocity() - a.velocity());
        double hood = a.hood() + t * (b.hood() - a.hood());
        return new double[] {vel, hood};
      }
    }

    // Fallback (shouldn't happen): return closest
    ShootData closest = data[0];
    double bestDiff = Math.abs(distance - closest.distance());
    for (ShootData sd : data) {
      double diff = Math.abs(distance - sd.distance());
      if (diff < bestDiff) {
        closest = sd;
        bestDiff = diff;
      }
    }
    return new double[] {closest.velocity(), closest.hood()};
  }

  private static final ShootData[] shootDatas = new ShootData[]{
    new ShootData(.6,3000,0.032),
    new ShootData(1.97, 3350, 0.065),
    new ShootData(2.7,3450,0.067),
    new ShootData(5.0, 3875,0.085)
  };

}
