// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class AutonomousManager {
  Drive m_Drive;
  ShooterSubsystem m_ShooterSubsystem;
  ClimbSubsystem m_ClimbSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  public AutonomousManager(
      Drive drive,
      ShooterSubsystem shooterSubsystem,
      ClimbSubsystem climbSubsystem,
      IntakeSubsystem intakeSubsystem) {
    m_Drive = drive;
    m_ClimbSubsystem = climbSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
  }

  public void initialize() {
    register("IntakeOn", Commands.runOnce(() -> m_IntakeSubsystem.out()));
    register("Shoot", new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_Drive));
    register("ShootStop", new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_Drive));
    register("ClimbUp", new ClimbCommand(m_ClimbSubsystem));
  }

  private void register(String name, Command command) {
    NamedCommands.registerCommand(name, command);
  }
}
