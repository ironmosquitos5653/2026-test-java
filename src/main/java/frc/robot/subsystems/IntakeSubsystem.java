// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int intakeMotorCANId = 0;
  private static final int intakeDeployMotorCANId = 9;

  private SparkFlex intakeMotor;
  private SparkFlex intakeDeployMotor;

  public IntakeSubsystem() {
    intakeMotor = new SparkFlex(intakeMotorCANId, MotorType.kBrushless);
    intakeDeployMotor = new SparkFlex(intakeDeployMotorCANId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setDeploySpeed(double speed) {
    intakeDeployMotor.set(-speed);
  }
}
