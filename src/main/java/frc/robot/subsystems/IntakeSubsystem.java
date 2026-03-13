// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int intakeMotorCANId = 16;
  private static final int intakeDeployMotorCANId = 9;

  private SparkFlex intakeMotor;
  private SparkFlex intakeDeployMotor;

  private DigitalInput intakeSensor = new DigitalInput(0);

  private double intakeOutPosition = -143;

  public IntakeSubsystem() {
    intakeMotor = new SparkFlex(intakeMotorCANId, MotorType.kBrushless);
    intakeDeployMotor = new SparkFlex(intakeDeployMotorCANId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake position", intakeMotor.getEncoder().getPosition());
    if (!intakeSensor.get()) {
      intakeMotor.getEncoder().setPosition(0);
    }
  }

  public void setDeploySpeed(double speed) {
    intakeDeployMotor.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    if (!intakeSensor.get()) {
      speed = 0;
    }
    intakeMotor.set(speed);
  }

  public boolean isOut() {
    return intakeMotor.getEncoder().getPosition() < intakeOutPosition;
  }

  public double getIntakeOutPosition() {
    return intakeOutPosition;
  }
}
