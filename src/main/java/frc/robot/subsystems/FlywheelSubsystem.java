// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

  private final SparkFlex flyWheelMotor;
  private final RelativeEncoder flyWheelEncoder;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem(SparkFlex motor) {
    flyWheelMotor = motor;
    flyWheelEncoder = flyWheelMotor.getEncoder();
  }
  // Fields (add to class)
  private double targetRPM = 0;

  // Tuning: kP is (percent output) per RPM when using percent control,
  // or (volts) per RPM when using voltage control with setVoltage().
  private double kP = 0.004; // example: 0.0005 percent-per-RPM (tune)
  private double kFF =
      1.05 * 12.0
          / 6784; // example feed-forward: fraction-per-RPM (for NEO maxRPM=5676). Use 12/5676 if
  // using volts-per-RPM.
  private final boolean useVoltageControl = true;

  // Add if you need to scale the kP value differently; tune carefully
  private final double outputDeadband =
      0.0; // leave 0 or set small minimum to overcome static friction

  // Example motor & encoder fields — replace with your actual types/fields
  // private final CANSparkMax flywheelMotor;
  // private final RelativeEncoder flywheelEncoder;

  // Public API
  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
  }

  public void stopFlywheel() {
    targetRPM = 0.0;
    // Optionally set motor to 0 immediately:
    // flywheelMotor.set(0.0);
  }

  // Utility to get current RPM from your encoder
  private double getCurrentRPM() {
    // Replace with your actual encoder call. For SparkMax RelativeEncoder: getVelocity() -> RPM
    // For TalonFX you'd need to convert native units/100ms to RPM
    return flyWheelEncoder.getVelocity();
  }

  // In your periodic() method, or a dedicated control loop, add:
  @Override
  public void periodic() {
    super.periodic();

    double currentRPM = getCurrentRPM();
    double error = targetRPM - currentRPM;

    // Compute controller output
    // If using percent output control: kP and kFF are fractions (-1..1)
    // If using voltage control with motor.setVoltage(), kP would be volts-per-RPM and kFF
    // volts-per-RPM
    double output;
    if (useVoltageControl) {
      // Example: kP (volts per RPM), kFF (volts per RPM)
      double pTerm = kP * error; // volts
      double ffTerm = kFF * targetRPM; // volts
      double volts = pTerm + ffTerm;
      // Clamp to allowed voltage range (-12..12)
      volts = Math.max(-12.0, Math.min(12.0, volts));
      // Apply voltage (if your API supports it). Example for SparkMax/WPILib:
      flyWheelMotor.setVoltage(volts);
      // flyWheelMotor.setVoltage(VoltageSupply.getVoltage() == 0 ? 0 : volts); // adapt as needed
    } else {
      // Percent output mode: kP and kFF are fraction per RPM
      double pTerm = kP * error; // fraction (-1..1 roughly)
      double ffTerm = kFF * targetRPM; // fraction
      output = pTerm + ffTerm;

      // Optional deadband compensation
      if (Math.abs(output) < outputDeadband && Math.abs(targetRPM) > 0.0) {
        output = Math.signum(output) * outputDeadband;
      }

      // Clamp between -1 and 1
      output = Math.max(-1.0, Math.min(1.0, output));

      // Apply to motor
      flyWheelMotor.set(output);
    }
  }
}
