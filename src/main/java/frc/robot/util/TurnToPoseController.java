package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Small helper controller that computes an angular velocity to rotate the robot so it faces
 * a target pose.
 *
 * Usage:
 *  - Construct with tuned PID gains.
 *  - Call calculate(currentPose, targetPose) each loop to get angular velocity (rad/s).
 *  - Optionally call atSetpoint() to know when the rotation is within tolerance.
 */
public class TurnToPoseController {
  private final PIDController pid;
  private double maxAngularVelocity = Double.POSITIVE_INFINITY;

  public TurnToPoseController(double kP, double kI, double kD) {
    pid = new PIDController(kP, kI, kD);
    // Enable continuous input for angle wrapping (-pi..pi)
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Set the maximum absolute angular velocity (rad/s) this controller will output.
   * If not set, the controller will not clamp its output.
   */
  public void setMaxAngularVelocity(double maxAngularVelocityRadPerSec) {
    this.maxAngularVelocity = Math.abs(maxAngularVelocityRadPerSec);
  }

  /**
   * Set tolerances (radians) for at-setpoint checking.
   */
  public void setTolerance(double positionToleranceRad, double velocityToleranceRadPerSec) {
    pid.setTolerance(positionToleranceRad, velocityToleranceRadPerSec);
  }

  /**
   * Calculate the angular velocity (rad/s) to rotate from currentPose to face the targetPose.
   * The sign follows the usual rotation convention: positive means counter-clockwise.
   */
  public double calculate(Pose2d currentPose, Pose2d targetPose) {
    Rotation2d toTarget =
        new Rotation2d(
            Math.atan2(
                targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));

    Rotation2d currentRot = currentPose.getRotation();

    double error = toTarget.minus(currentRot).getRadians();

    double output = pid.calculate(error);

    // Clamp
    if (Double.isFinite(maxAngularVelocity)) {
      if (output > maxAngularVelocity) {
        output = maxAngularVelocity;
      } else if (output < -maxAngularVelocity) {
        output = -maxAngularVelocity;
      }
    }

    return output;
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public void reset() {
    pid.reset();
  }

  /*
   *   double omega = drive.getTurnToPoseOutput(target, controller);
    // No translation, rotate in place using chassis speeds
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

   */
}
