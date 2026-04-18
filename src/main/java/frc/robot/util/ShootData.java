package frc.robot.util;

public record ShootData(double distance, double velocity, double hood) {
  public static final ShootData[] hubShootDatas =
      new ShootData[] {
        new ShootData(.6, 2600, 0.035),
        new ShootData(2.18, 3000, 0.06),
        new ShootData(3.46, 3225, 0.073),
        new ShootData(5.0, 3855, 0.085)
      };
  public static final ShootData[] oldShootDatas =
      new ShootData[] {
        new ShootData(.6, 3000, 0.032),
        new ShootData(2.18, 3100, 0.05),
        new ShootData(3.46, 3425, 0.067),
        new ShootData(5.0, 3855, 0.085)
      };
  public static final ShootData[] backFieldShootDatas =
      new ShootData[] {
        new ShootData(.6, 3000, 0.1),
        // new ShootData(1.18, 3100, 0.05),
        // new ShootData(2.46, 3425, 0.067),
        new ShootData(15.0, 7000, 0.165)
      };
}
