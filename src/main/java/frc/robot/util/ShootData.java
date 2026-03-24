package frc.robot.util;

public record ShootData(double distance, double velocity, double hood) {
      public static final ShootData[] shootDatas =
      new ShootData[] {
        new ShootData(.5, 3000, 0.032),
        new ShootData(1.18, 3100, 0.05),
        new ShootData(2.46, 3425, 0.067),
        new ShootData(4.0, 3855, 0.085)
      };
}
