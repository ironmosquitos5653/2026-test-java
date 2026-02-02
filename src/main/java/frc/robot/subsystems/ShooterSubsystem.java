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
  private static final int shooter3MotorCANId = 13;
  private static final int advance1MotorCANId = 14;
  private static final int advance2MotorCANId = 15;

  private SparkFlex shooter1Motor;
  private SparkFlex shooter2Motor;
  private SparkFlex shooter3Motor;
  private SparkFlex advance1Motor;
  private SparkFlex advance2Motor;

  public ShooterSubsystem() {
    shooter1Motor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);
    shooter2Motor = new SparkFlex(shooter2MotorCANId, MotorType.kBrushless);
    shooter3Motor = new SparkFlex(shooter3MotorCANId, MotorType.kBrushless);
    advance1Motor = new SparkFlex(advance1MotorCANId, MotorType.kBrushless);
    advance2Motor = new SparkFlex(advance2MotorCANId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {}

  public void setShootSpeed(double speed) {
    shooter1Motor.set(speed);
    shooter2Motor.set(speed);
    shooter3Motor.set(speed);
  }
}
