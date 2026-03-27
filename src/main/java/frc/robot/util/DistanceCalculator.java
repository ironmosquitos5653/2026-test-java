package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;

public class DistanceCalculator {

  public static Pose2d redTarget = new Pose2d(new Translation2d(11.9, 4.0), new Rotation2d(0));
  public static Pose2d blueTarget = new Pose2d(new Translation2d(4.6, 4.0), new Rotation2d(0));

  private final Drive m_drive;

  public DistanceCalculator(Drive drive) {
        m_drive = drive;
  }


public boolean isBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
}

public Pose2d getTargetPose2d() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redTarget;
    }
    return blueTarget;
  }

  public double getDistance() {
    return getDistance(getTargetPose2d());
  }

  public double getDistance(Pose2d pose) {
    return m_drive.getPose().getTranslation().getDistance(pose.getTranslation());
  }

  public double[] getVelocityAndHood() {
    return getVelocityAndHood(getTargetPose2d());
  }

  public double[] getVelocityAndHood(Pose2d pose) {
    return getVelocityAndHood(pose, ShootData.hubShootDatas);
  }

  public double[] getVelocityAndHood(Pose2d pose, ShootData[] data) {
    double distance = getDistance(pose);
    return getVelocityAndHood(distance, data);
  }
  public double[] getVelocityAndHood(double distance) {
    ShootData[] data = ShootData.hubShootDatas;
    return getVelocityAndHood(distance, data);
  }
  public double[] getVelocityAndHood(double distance, ShootData[] data) {
    // Use shootDatas to interpolate (or extrapolate) velocity and hood
    // If distance is below the first entry, extrapolate using first two
    if (distance <= data[0].distance()) {
      ShootData a = data[0];
      ShootData b = data[1];
      double t = (distance - a.distance()) / (b.distance() - a.distance());
      if (Double.isNaN(t) || Double.isInfinite(t)) t = 0.0;
      double vel = a.velocity() + t * (b.velocity() - a.velocity());
      double hood = a.hood() + t * (b.hood() - a.hood());
      return new double[] {vel, hood};
    }

    // If distance is above the last entry, extrapolate using last two
    int last = data.length - 1;
    if (distance >= data[last].distance()) {
      ShootData a = data[last - 1];
      ShootData b = data[last];
      double t = (distance - a.distance()) / (b.distance() - a.distance());
      if (Double.isNaN(t) || Double.isInfinite(t)) t = 0.0;
      double vel = a.velocity() + t * (b.velocity() - a.velocity());
      double hood = a.hood() + t * (b.hood() - a.hood());
      return new double[] {vel, hood};
    }

    // Otherwise find the interval [i, i+1] that contains distance
    for (int i = 0; i < data.length - 1; i++) {
      ShootData a = data[i];
      ShootData b = data[i + 1];
      if (distance >= a.distance() && distance <= b.distance()) {
        double t = (distance - a.distance()) / (b.distance() - a.distance());
        if (Double.isNaN(t) || Double.isInfinite(t)) t = 0.0;
        double vel = a.velocity() + t * (b.velocity() - a.velocity());
        double hood = a.hood() + t * (b.hood() - a.hood());
        return new double[] {vel, hood};
      }
    }

    // Fallback (shouldn't happen): return closest
    ShootData closest = data[0];
    double bestDiff = Math.abs(distance - closest.distance());
    for (ShootData sd : data) {
      double diff = Math.abs(distance - sd.distance());
      if (diff < bestDiff) {
        closest = sd;
        bestDiff = diff;
      }
    }
    return new double[] {closest.velocity(), closest.hood()};
  }

}
