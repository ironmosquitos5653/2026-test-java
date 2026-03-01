// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int intakeMotorCANId = 16;
  private static final int intakeDeployMotorCANId = 9;

  private SparkFlex intakeMotor;
  private SparkFlex intakeDeployMotor;
  private double speed = .5;

  public IntakeSubsystem() {
    intakeMotor = new SparkFlex(intakeMotorCANId, MotorType.kBrushless);
    intakeDeployMotor = new SparkFlex(intakeDeployMotorCANId, MotorType.kBrushless);
    // setCurrentLimit(60);
  }

  @Override
  public void periodic() {
    if (intakeDeployMotor.getEncoder().getVelocity() != 0) {
      // setCurrentLimit(50);
    } else {
      // setCurrentLimit(20);
    }
  }

  public void setCurrentLimit(int Current) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(Current);
    intakeDeployMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void in() {
    // intakeDeployMotor.set(1);
    intakeMotor.set(0);
  }

  public void out() {
    // intakeDeployMotor.set(-1);
    intakeMotor.set(-1);
  }

  public void toggle() {
    if (intakeMotor.get() != 0) {
      in();
    } else {
      out();
    }
  }

  public void setDeploySpeed(double speed) {
    intakeDeployMotor.set(speed);
  }
}
