package frc.robot;

import frc.robot.commands.ShootCommand;

public class Test {
  public static void main(String... args) {
    double distance = .5;
    double[] values = ShootCommand.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 1;
    values = ShootCommand.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 2;
    values = ShootCommand.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 2.7;
    values = ShootCommand.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 3;
    values = ShootCommand.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 4;
    values = ShootCommand.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);
  }
}
