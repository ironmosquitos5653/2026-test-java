package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public static Pose2d redTarget = new Pose2d(new Translation2d(11.9, 4.6), new Rotation2d(0));
  public static Pose2d blueTarget = new Pose2d(new Translation2d(4.6, 4.0), new Rotation2d(0));

  private Transform3d leftCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(14), Units.inchesToMeters(11), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(15)));

  private Transform3d rightCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(14), Units.inchesToMeters(-11), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(-15)));

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
    tab.addString("leftCamera", this::getLeftPosition).withPosition(0, 1).withSize(6, 0);
    tab.addString("rightCamera", this::getRightPosition).withPosition(0, 2).withSize(6, 0);

    tab.add("Field", field2d).withPosition(3, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    updateCameras();
    double distance = PhotonUtils.getDistanceToPose(getTargetPose2d(), m_driveSubsystem.getPose());
    SmartDashboard.putNumber("DISTNCE", distance);
  }

  int count = 0;

  public void updateCameras() {
    leftPose2d = updateCamera(leftCamera, leftCameraTransform);
    if (leftPose2d != null) {
      leftPosition = getFomattedPose(leftPose2d);
      SmartDashboard.putString("LEft:" + fiducial, leftPosition);
    }

    rightPose2d = updateCamera(rightCamera, rightCameraTransform);
    if (rightPose2d != null) {
      rightPosition = getFomattedPose(rightPose2d);
      SmartDashboard.putString("Right:" + fiducial, rightPosition);
    }
    updateField();
    count++;
  }

  private Pose2d updateCamera(PhotonCamera camera, Transform3d cameraTransform) {

    Pose2d latest = null;
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (var result : results) {
      if (result.hasTargets()) {
        PhotonTrackedTarget tar = result.getBestTarget();

        // Optional<MultiTargetPNPResult> tar = result.getMultiTagResult();

        // SmartDashboard.putBoolean("CAM" + camera.getName(), tar.isPresent());

        if (true) { // tar.isPresent()) {
          // PnpResult px = tar.get().estimatedPose;

          // Transform3d robot = tar.get().estimatedPose.best.plus(cameraTransform);
          // Pose2d p =
          //   new Pose2d(robot.getX(), robot.getY(), new Rotation2d(robot.getRotation().getZ()));

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

            fiducial = tar.getFiducialId() + "";
            if (tagPose.isEmpty() || tar.getPoseAmbiguity() > 0.15) {
              continue;
            }
            Pose2d p =
                PhotonUtils.estimateFieldToRobotAprilTag(
                        c2t, tagPose.get(), cameraTransform.inverse())
                    .toPose2d();

            Pose2d tag = fieldLayout.getTagPose(tar.getFiducialId()).get().toPose2d();
            double distance = PhotonUtils.getDistanceToPose(p, tag);

            double std = distance * distance * .1;

            SmartDashboard.putNumber("STD", std);

            SmartDashboard.putString(
                camera.getName() + "-cam",
                p.getX()
                    + " - "
                    + p.getY()
                    + " - "
                    + Units.radiansToDegrees(p.getRotation().getRadians()));

            latest = p;
            m_driveSubsystem.addVisionMeasurement(
                p, result.getTimestampSeconds(), VecBuilder.fill(std, std, std));
          }
        }
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

  String leftPosition = "(N/A)";

  public String getLeftPosition() {
    return leftPosition;
  }

  String rightPosition = "(N/A)";
  String fiducial = "nope";

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

  public static Pose2d getTargetPose2d() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redTarget;
    }
    return blueTarget;
  }
}
