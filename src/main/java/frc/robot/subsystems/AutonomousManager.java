// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FirstShootCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SimpleShootAndIntakeCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Aimer;
import frc.robot.util.DistanceCalculator;

/** Add your docs here. */
public class AutonomousManager {
  Drive m_Drive;
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  DistanceCalculator m_DistanceCalculator;
  Aimer m_aimer;

  public AutonomousManager(
      Drive drive,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      DistanceCalculator distanceCalculator,
      Aimer aimer) {
    m_Drive = drive;

    m_IntakeSubsystem = intakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_DistanceCalculator = distanceCalculator;
    m_aimer = aimer;

    initialize();
  }

  public void initialize() {
    register("IntakeOn", new IntakeDeployCommand(m_IntakeSubsystem));
    register(
        "ShootTimed",
        new ShootCommand(
            m_ShooterSubsystem, m_IntakeSubsystem, m_Drive, m_DistanceCalculator, m_aimer, 4));
    register(
        "Shoot",
        new ShootCommand(
            m_ShooterSubsystem, m_IntakeSubsystem, m_Drive, m_DistanceCalculator, m_aimer, true));
    register(
        "ShootNmove",
        new SimpleShootAndIntakeCommand(
            m_ShooterSubsystem, m_IntakeSubsystem, m_Drive, m_DistanceCalculator, true));
    register(
        "Shimtake",
        new SimpleShootAndIntakeCommand(
            m_ShooterSubsystem, m_IntakeSubsystem, m_Drive, m_DistanceCalculator, false));
    register(
        "ShootOff",
        Commands.runOnce(
            () -> SimpleShootAndIntakeCommand.finished = true,
            m_ShooterSubsystem,
            m_IntakeSubsystem));
    register(
        "FirstShoot", new FirstShootCommand(m_ShooterSubsystem, m_Drive, m_DistanceCalculator));
    register(
        "IntakeIn",
        Commands.startEnd(
                () -> m_IntakeSubsystem.setDeploySpeed(0),
                () -> m_IntakeSubsystem.setDeploySpeed(0),
                m_IntakeSubsystem)
            .withTimeout(.5));
    register(
        "DeployClimb",
        Commands.runOnce(() -> m_ShooterSubsystem.deployClimb(), m_ShooterSubsystem));
    register(
        "Climb", Commands.runOnce(() -> m_ShooterSubsystem.setHoodPosition(0), m_ShooterSubsystem));

    new EventTrigger("IntakeOn").onTrue(new IntakeDeployCommand(m_IntakeSubsystem));
  }

  private void register(String name, Command command) {
    System.out.println(">>> " + name + " - " + command);
    NamedCommands.registerCommand(name, command);
  }
}
