package frc.robot.util;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class Aimer {

  private final TurnToPoseController turnController = new TurnToPoseController(1, 0, .1);
  private final Drive m_drive;
  private final DistanceCalculator m_DistanceCalculator;

  private Pose2d target;

  public Aimer(Drive drive, DistanceCalculator distanceCalculator) {
    m_drive = drive;
    m_DistanceCalculator = distanceCalculator;
    distanceCalculator.m_aimer = this;
  }

  public void setTarget(Pose2d t) {
    target = t;
    if (target != null) {
      PPHolonomicDriveController.overrideRotationFeedback(this::calculate);
    } else {
      PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
  }

  public boolean hasTarget() {
    return target != null;
  }

  public double calculate() {
    return turnController.calculate(m_drive.getPose(), target);
  }

  private Pose2d blueLeft = new Pose2d(.5, 7.5, null);
  private Pose2d blueRight = new Pose2d(.5, 0.5, null);
  private Pose2d redLeft = new Pose2d(16, .5, null);
  private Pose2d redRight = new Pose2d(16, 7.5, null);

  public boolean isBackField() {
    Pose2d currentPose2d = m_drive.getPose();
    boolean isBlue = m_DistanceCalculator.isBlue();
    return ((isBlue && currentPose2d.getX() < 4.9) || (!isBlue && currentPose2d.getX() > 11.7));
  }

  public Pose2d getTargetPose() {
    Pose2d currentPose2d = m_drive.getPose();
    boolean isBackField = isBackField();
    if (m_DistanceCalculator.isBlue()) {
      if (isBackField) {
        return m_DistanceCalculator.getTargetPose2d();
      }
      if (currentPose2d.getY() < 4) {
        return blueRight;
      } else {
        return blueLeft;
      }
    } else {
      if (isBackField) {
        return m_DistanceCalculator.getTargetPose2d();
      }
      if (currentPose2d.getY() < 4) {
        return redLeft;
      } else {
        return redRight;
      }
    }
  }
}
