// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DistanceCalculator;
import frc.robot.util.ShootData;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleShootAndIntakeCommand extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  DistanceCalculator m_DistanceCalculator;
  Drive m_drive;

  public static boolean finished = false;
  private final boolean m_shootAtHub;
  private Pose2d m_targetPose;

  /** Creates a new ShootCommand. */
  public SimpleShootAndIntakeCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      DistanceCalculator distanceCalculator,
      boolean shootAtHub) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_drive = drive;
    m_DistanceCalculator = distanceCalculator;
    m_shootAtHub = shootAtHub;
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SHOOTING!!!");
    System.out.println("RainAndWalterAreAWESOME...veryAwesome");
    m_targetPose = getTargetPose();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] calculations = null;
    if (m_shootAtHub) {
      calculations = m_DistanceCalculator.getVelocityAndHood();
    } else {
      calculations =
          m_DistanceCalculator.getVelocityAndHood(m_targetPose, ShootData.backFieldShootDatas);
    }
    double velocity = calculations[0];
    double hood = calculations[1];

    setHood(hood);
    setShootSpeed(velocity);

    if (m_ShooterSubsystem.getVelocity() < -velocity * .90) {
      m_ShooterSubsystem.setAdvanceSpeed(1);
    }

    m_IntakeSubsystem.setIntakeSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShootSpeed(0);
    m_ShooterSubsystem.setAdvanceSpeed(0);
    m_ShooterSubsystem.setHoodPosition(HoodPosition.DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  public void setHood(double position) {
    m_ShooterSubsystem.setHoodPosition(position);
  }

  public void setShootSpeed(double velocity) {
    m_ShooterSubsystem.setShootSpeed(velocity);
  }

  private Pose2d blueLeft = new Pose2d(.5, 6.5, null);
  private Pose2d blueRight = new Pose2d(.5, 1.5, null);
  private Pose2d redLeft = new Pose2d(16, 1.5, null);
  private Pose2d redRight = new Pose2d(16, 6.5, null);

  public Pose2d getTargetPose() {
    if (m_shootAtHub) {
      return m_DistanceCalculator.getTargetPose2d();
    } else {
      Pose2d currentPose2d = m_drive.getPose();
      if (currentPose2d.getX() < 8) {
        if (currentPose2d.getY() < 4) {
          return blueRight;
        } else {
          return blueLeft;
        }
      } else {
        if (currentPose2d.getY() < 4) {
          return redRight;
        } else {
          return redLeft;
        }
      }
    }
  }
}
