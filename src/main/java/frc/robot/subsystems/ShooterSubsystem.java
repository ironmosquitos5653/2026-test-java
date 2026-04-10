// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static final int shooterMotorCANId = 10;

  private static final int advance1MotorCANId = 13;
  private static final int advance2MotorCANId = 14;
  private static final int hoodRotateMotorCANId = 17;

  private SparkFlex shooter1Motor;

  private SparkFlex advance1Motor;
  private SparkFlex advance2Motor;

  private SparkFlex hoodRotateMotor;
  private PIDController hoodEncoderPidController;
  private AbsoluteEncoder hoodEncoder = null;

  FlywheelSubsystem fws1;

  public static enum HoodPosition {
    DOWN(0.019),
    DEPLOY_CLIMB(.165),
    CLIMB(.056),
    MAX(0.165);

    public final double value;

    HoodPosition(double v) {
      value = v;
    }
  };

  private double hoodPosition = HoodPosition.DOWN.value;

  public ShooterSubsystem() {
    shooter1Motor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);

    advance1Motor = new SparkFlex(advance1MotorCANId, MotorType.kBrushless);
    advance2Motor = new SparkFlex(advance2MotorCANId, MotorType.kBrushless);

    hoodRotateMotor = new SparkFlex(hoodRotateMotorCANId, MotorType.kBrushless);

    fws1 = new FlywheelSubsystem(shooter1Motor);

    hoodEncoderPidController = new PIDController(12, 0, .5);
    hoodEncoderPidController.enableContinuousInput(0, 1);

    hoodEncoder = hoodRotateMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM1", shooter1Motor.getEncoder().getVelocity());

    hoodEncoderPidController.setSetpoint(hoodPosition);
    double speed = hoodEncoderPidController.calculate(hoodEncoder.getPosition());
    hoodRotateMotor.set(speed);
    SmartDashboard.putNumber("Hood Encoder", hoodEncoder.getPosition());
    boolean hasHood = hoodEncoder.getPosition() != 0;
    SmartDashboard.putBoolean("HoodON", hasHood);
    if (!hasHood) {
      System.out.println("HoodOff");
    }
  }

  public void setShootSpeed(double speed) {
    fws1.setTargetRPM(-speed);
  }

  public void setAdvanceSpeed(double speed) {
    advance1Motor.set(speed);
    advance2Motor.set(speed);
  }

  public void setHoodPosition(HoodPosition position) {
    setHoodPosition(position.value);
  }

  public void setHoodPosition(double position) {
    // do not let it go below bottom.
    if (hoodPosition < HoodPosition.DOWN.value) {
      position = HoodPosition.DOWN.value;
    }

    hoodPosition = position;
  }

  public void dump() {
    SmartDashboard.putNumber("RPM 1 start", shooter1Motor.getEncoder().getVelocity());
  }

  public boolean indexStuck() {
    return (advance1Motor.getEncoder().getVelocity() < 100
            && advance1Motor.getEncoder().getVelocity() > -100)
        || (advance2Motor.getEncoder().getVelocity() < 100
            && advance2Motor.getEncoder().getVelocity() > -100);
  }

  public double getVelocity() {
    return shooter1Motor.getEncoder().getVelocity();
  }

  public double getShooterVelocity(int shooter) {
    return shooter1Motor.getEncoder().getVelocity();
  }

  public void deployClimb() {
    setHoodPosition(HoodPosition.DEPLOY_CLIMB);
  }

  public void climb() {
    setHoodPosition(HoodPosition.CLIMB);
  }
}
