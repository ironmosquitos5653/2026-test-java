// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

 private static final int climbMotorCANId = 0;
 private static final int climb2MotorCANId = 0;

 private SparkFlex climbMotor;
 private SparkFlex climb2Motor;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotor = new SparkFlex(climbMotorCANId, MotorType.kBrushless);
    climb2Motor = new SparkFlex(climb2MotorCANId, MotorType.kBrushless);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   public void setSpeed(double speed) {
    climbMotor.set(speed);
    climb2Motor.set(-speed);
  }
}