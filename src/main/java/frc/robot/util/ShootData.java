package frc.robot.util;

public record ShootData(double distance, double velocity, double hood) {
  public static final ShootData[] hubShootDatas =
      new ShootData[] {
        new ShootData(.6, 2600, 0.035),
        new ShootData(2.18, 3000, 0.06),
        new ShootData(3.46, 3225, 0.073),
        new ShootData(4, 3350, 0.075),
        new ShootData(5.0, 3500, 0.105)
      };
  /*
  public static final ShootData[] oldShootDatas =
      new ShootData[] {
        new ShootData(1.26, 2500, 0.032),
        new ShootData(2.5, 3100, 0.05),
        new ShootData(3.5, 3400, 0.067),
        new ShootData(4, 3460, 0.068),
        new ShootData(5.0, 3855, 0.085)
      };
      */
  public static final ShootData[] backFieldShootDatas =
      new ShootData[] {
        new ShootData(.6, 2500, 0.145),
        // new ShootData(1.18, 3100, 0.05),
        // new ShootData(2.46, 3425, 0.067),
        new ShootData(15.0, 7000, 0.165)
      };
}
