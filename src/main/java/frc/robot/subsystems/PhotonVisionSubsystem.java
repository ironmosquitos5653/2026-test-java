package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
  private Drive m_driveSubsystem;
  /** Creates a new VisionSubsystem. */
  private final Field2d field2d = new Field2d();

  private final String leftCameraName = "leftCamera";
  private final String rightCameraName = "rightCamera";

  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;

  private Transform3d leftCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-14), Units.inchesToMeters(-12.5), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(165)));

  private Transform3d rightCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-14), Units.inchesToMeters(12.5), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(-165)));

  private Pose2d leftPose2d;
  private Pose2d rightPose2d;

  AprilTagFieldLayout fieldLayout;

  public PhotonVisionSubsystem(Drive driveSubsystem) {

    leftCamera = new PhotonCamera(leftCameraName);
    rightCamera = new PhotonCamera(rightCameraName);

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    m_driveSubsystem = driveSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    tab.addString("robot", this::getFomattedPose).withPosition(0, 0).withSize(4, 0);
    tab.addString("leftCamera", this::getLeftPosition).withPosition(0, 1).withSize(8, 0);
    tab.addString("rightCamera", this::getRightPosition).withPosition(0, 2).withSize(8, 0);

    tab.add("Field", field2d).withPosition(3, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    updateCameras();
  }

  int count = 0;

  public void updateCameras() {
    leftPose2d = updateCamera(leftCamera, leftCameraTransform);
    if (leftPose2d != null) leftPosition = getFomattedPose(leftPose2d);

    rightPose2d = updateCamera(rightCamera, rightCameraTransform);
    if (rightPose2d != null) rightPosition = getFomattedPose(rightPose2d);
    updateField();
    count++;
  }

  private Pose2d updateCamera(PhotonCamera camera, Transform3d cameraTransform) {

    Pose2d latest = null;
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (var result : results) {
      PhotonTrackedTarget tar = result.getBestTarget();

      if (tar != null) {
        Transform3d c2t = tar.getBestCameraToTarget();
        SmartDashboard.putString(
            camera.getName() + "-cam1Target",
            c2t.getX()
                + " - "
                + c2t.getY()
                + " - "
                + c2t.getZ()
                + "  -  "
                + Units.radiansToDegrees(c2t.getRotation().getAngle()));
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tar.getFiducialId());
        if (tagPose.isEmpty() || tar.getPoseAmbiguity() > 0.15) {
          continue;
        }
        Pose2d p =
            PhotonUtils.estimateFieldToRobotAprilTag(c2t, tagPose.get(), cameraTransform.inverse())
                .toPose2d();

        SmartDashboard.putString(
            camera.getName() + "-cam",
            p.getX()
                + " - "
                + p.getY()
                + " - "
                + Units.radiansToDegrees(p.getRotation().getRadians()));

        latest = p;
        m_driveSubsystem.addVisionMeasurement(
            p, result.getTimestampSeconds(), VecBuilder.fill(.0, .1, .1));
      }
    }
    return latest;
  }

  public Pose2d cameraTransform(Pose2d pose, Transform3d cameraTransform) {
    return new Pose3d(pose).transformBy(cameraTransform).toPose2d();
  }

  public void updateField() {
    field2d.setRobotPose(m_driveSubsystem.getPose());
  }

  String leftPosition;

  public String getLeftPosition() {
    return leftPosition = "(N/A)";
  }

  String rightPosition = "(N/A)";

  public String getRightPosition() {
    return rightPosition;
  }

  private String getFomattedPose() {
    return getFomattedPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose(Pose2d pose) {

    if (pose == null) {
      return "(N/A)";
    }

    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
