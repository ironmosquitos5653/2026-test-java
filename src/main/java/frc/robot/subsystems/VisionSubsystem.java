// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private Drive m_driveSubsystem;
  /** Creates a new VisionSubsystem. */
  private final Field2d field2d = new Field2d();

  private final String reefCamera = "limelight-right";
  private final String coralStationCamera = "limelight-coral";

  private final PhotonCamera leftCamera = new PhotonCamera("Left_Camera");
  private final PhotonCamera rightCamera = new PhotonCamera("Right_Camera");

  private final ArrayList<Integer> coralStationTags;
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  private Transform3d rightTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));
  private Transform3d leftTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));

  PhotonPoseEstimator leftPhotonPoseEstimator =
      new PhotonPoseEstimator(
          fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftTransform);
  PhotonPoseEstimator rightPhotonPoseEstimator =
      new PhotonPoseEstimator(
          fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightTransform);

  public VisionSubsystem(Drive driveSubsystem) {

    m_driveSubsystem = driveSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(4, 0);

    tab.addString("Pose2", this::getFomattedPose2).withPosition(0, 3).withSize(2, 0);
    tab.add("Field", field2d).withPosition(3, 0).withSize(6, 4);
    coralStationTags = new ArrayList<>();

    coralStationTags.add(1);
    coralStationTags.add(2);
    coralStationTags.add(12);
    coralStationTags.add(13);
  }

  @Override
  public void periodic() {
    updateCamera();
  }

  public void updateCamera() {
    var leftResult = leftCamera.getLatestResult();
    var rightResult = rightCamera.getLatestResult();
    // Correct pose estimate with vision measurements

    if (leftResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = leftResult.getTargets();
    }

    updateField();
  }

  public Pose2d cameraTransform(Pose2d pose, Transform3d cameraTransform) {
    return new Pose3d(pose).transformBy(cameraTransform).toPose2d();
  }

  public void updateField() {
    field2d.setRobotPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose() {
    return getFomattedPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose2() {
    return getFomattedPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose(Pose2d pose) {

    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
