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

  private static final int shooterMotorCANId = 10;
  private static final int shooter2MotorCANId = 11;
  private static final int shooter3MotorCANId = 12;
  private static final int advance1MotorCANId = 13;
  private static final int advance2MotorCANId = 14;
  private static final int hoodRotateMotorCANId = 17;

  private SparkFlex shooter1Motor;
  private SparkFlex shooter2Motor;
  private SparkFlex shooter3Motor;
  private SparkFlex advance1Motor;
  private SparkFlex advance2Motor;
  private SparkFlex hoodRotateMotor;
  private PIDController hoodEncoderPidController;
  private AbsoluteEncoder hoodEncoder = null;
  private SparkClosedLoopController pid1;
  private SparkClosedLoopController pid2;
  private SparkClosedLoopController pid3;
  private SparkMaxConfig config = new SparkMaxConfig();

  private boolean shooting = false;
  private double hoodPosition = .0;

  public ShooterSubsystem() {
    shooter1Motor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);
    shooter2Motor = new SparkFlex(shooter2MotorCANId, MotorType.kBrushless);
    shooter3Motor = new SparkFlex(shooter3MotorCANId, MotorType.kBrushless);
    advance1Motor = new SparkFlex(advance1MotorCANId, MotorType.kBrushless);
    advance2Motor = new SparkFlex(advance2MotorCANId, MotorType.kBrushless);
    hoodRotateMotor = new SparkFlex(hoodRotateMotorCANId, MotorType.kBrushless);

    pid1 = shooter1Motor.getClosedLoopController();
    pid2 = shooter2Motor.getClosedLoopController();
    pid3 = shooter3Motor.getClosedLoopController();
    config.closedLoop.p(.0001);
    config.closedLoop.i(.00000067);
    config.closedLoop.d(0);
    shooter1Motor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    shooter2Motor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    shooter3Motor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // config.closedLoop.ff(.0001);
    pid1.setSetpoint(0, ControlType.kVelocity);
    pid2.setSetpoint(0, ControlType.kVelocity);
    pid3.setSetpoint(0, ControlType.kVelocity);

    hoodEncoderPidController = new PIDController(3, 0, 0);
    hoodEncoderPidController.enableContinuousInput(0, 1);

    hoodEncoder = hoodRotateMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM1", shooter1Motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("RPM2", shooter2Motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("RPM3", shooter3Motor.getEncoder().getVelocity());

    hoodEncoderPidController.setSetpoint(hoodPosition);
    double speed = hoodEncoderPidController.calculate(hoodEncoder.getPosition());
    hoodRotateMotor.set(speed);
    SmartDashboard.putNumber("HoodSpeed", speed);
    SmartDashboard.putNumber("Hood Encoder", hoodEncoder.getPosition());
  }

  public void setShootSpeed(double speed) {
    shooting = speed != 0;
    pid1.setSetpoint(-speed, ControlType.kVelocity);
    pid2.setSetpoint(-speed, ControlType.kVelocity);
    pid3.setSetpoint(-speed, ControlType.kVelocity);
  }

  public void setAdvanceSpeed(double speed) {
    advance1Motor.set(speed);
    advance2Motor.set(speed);
  }

  public void setHoodPosition(double position) {
    hoodPosition = position;
  }

  public void setHoodSpeed(double speed) {
    hoodRotateMotor.set(speed);
  }

  public void dump() {
    SmartDashboard.putNumber("RPM 1 start", shooter1Motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("RPM 2 start", shooter2Motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("RPM 3 start", shooter3Motor.getEncoder().getVelocity());
  }
}
