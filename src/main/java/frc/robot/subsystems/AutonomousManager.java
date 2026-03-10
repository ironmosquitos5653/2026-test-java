// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FirstShootCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class AutonomousManager {
  Drive m_Drive;
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  public AutonomousManager(
      Drive drive, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_Drive = drive;

    m_IntakeSubsystem = intakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;

    initialize();
  }

  public void initialize() {
    register("IntakeOn", new IntakeDeployCommand(m_IntakeSubsystem));
    register("Shoot", new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_Drive, true));
    register("FirstShoot", new FirstShootCommand(m_ShooterSubsystem, m_Drive));

    new EventTrigger("IntakeOn").onTrue(new IntakeDeployCommand(m_IntakeSubsystem));
  }

  private void register(String name, Command command) {
    System.out.println(">>> " + name + " - " + command);
    NamedCommands.registerCommand(name, command);
  }
}
