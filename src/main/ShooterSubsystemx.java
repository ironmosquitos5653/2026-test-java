// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static final int shooterMotorCANId = 11;
  // private static final int shooter2MotorCANId = 12;
  private static final int hoodRotateMotorCANId = 13;

  private SparkFlex shooterMotor;
  private SparkFlex hoodRotateMotor;
  private PIDController hoodEncoderPidController;
  private AbsoluteEncoder hoodEncoder = null;
  private SparkClosedLoopController pid;
  private SparkMaxConfig config = new SparkMaxConfig();

  public ShooterSubsystem() {
    shooterMotor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);
    // shooter2Motor = new SparkFlex(shooter2MotorCANId, MotorType.kBrushless);
    hoodRotateMotor = new SparkFlex(hoodRotateMotorCANId, MotorType.kBrushless);
    pid = shooterMotor.getClosedLoopController();
    config.closedLoop.p(.0001);
    config.closedLoop.i(.000001);
    config.closedLoop.d(0);
    shooterMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    // config.closedLoop.ff(.0001);
    pid.setSetpoint(0, ControlType.kVelocity);

    hoodEncoderPidController = new PIDController(.25, 0, 0);
    hoodEncoderPidController.enableContinuousInput(0, 1);

    hoodEncoder = hoodRotateMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", shooterMotor.getEncoder().getVelocity());

    hoodEncoderPidController.setSetpoint(.5);
    double speed = hoodEncoderPidController.calculate(hoodEncoder.getPosition());
    SmartDashboard.putNumber("HoodSpeed", speed);
    SmartDashboard.putNumber("Hood Encoder", hoodEncoder.getPosition());
  }

  public void setSpeed1(double speed) {
    pid.setSetpoint(-speed, ControlType.kVelocity);
  }

  public void setSpeed2(double speed) {
    // hoodRotateMotor.set(speed);
  }

  public void setCurrentLimit(int current) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(current);
    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
