// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static final int shooterMotorCANId = 11;
  private static final int shooter2MotorCANId = 12;

  private SparkFlex shooterMotor;
  private SparkFlex shooter2Motor;

  public ShooterSubsystem() {
    shooterMotor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);
    shooter2Motor = new SparkFlex(shooter2MotorCANId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {}

  public void setSpeed1(double speed) {
    shooterMotor.set(speed);
  }

  public void setSpeed2(double speed) {
    shooter2Motor.set(-speed);
  }
}
