// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
  private double hoodPosition = .019;

  FlywheelSubsystem fws1;
  FlywheelSubsystem fws2;
  FlywheelSubsystem fws3;

  public ShooterSubsystem() {
    shooter1Motor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);
    shooter2Motor = new SparkFlex(shooter2MotorCANId, MotorType.kBrushless);
    shooter3Motor = new SparkFlex(shooter3MotorCANId, MotorType.kBrushless);
    advance1Motor = new SparkFlex(advance1MotorCANId, MotorType.kBrushless);
    advance2Motor = new SparkFlex(advance2MotorCANId, MotorType.kBrushless);
    hoodRotateMotor = new SparkFlex(hoodRotateMotorCANId, MotorType.kBrushless);

    fws1 = new FlywheelSubsystem(shooter1Motor);
    fws2 = new FlywheelSubsystem(shooter2Motor);
    fws3 = new FlywheelSubsystem(shooter3Motor);

    hoodEncoderPidController = new PIDController(12, 0, .5);
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
    fws1.setTargetRPM(-speed);
    fws2.setTargetRPM(-speed);
    fws3.setTargetRPM(-speed);
    shooting = speed != 0;
  }

  public void setAdvanceSpeed(double speed) {
    advance1Motor.set(speed);
    advance2Motor.set(speed);
  }

  public void setHoodPosition(double position) {
    if (position < .019) {
      position = .019;
    }
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

  public boolean isHoodDown() {
    return hoodEncoder.getPosition() < .02 || hoodEncoder.getPosition() > .9;
  }

  public double getVelocity() {
    return shooter2Motor.getEncoder().getVelocity();
  }

  public double getShooterVelocity(int shooter) {
    switch (shooter) {
      case 0:
        return shooter1Motor.getEncoder().getVelocity();
      case 1:
        return shooter2Motor.getEncoder().getVelocity();
      case 2:
        return shooter3Motor.getEncoder().getVelocity();
      default:
        return 0.0;
    }
  }
}
