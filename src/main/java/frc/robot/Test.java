package frc.robot;

import frc.robot.util.DistanceCalculator;

public class Test {
  public static void main(String... args) {
    DistanceCalculator distanceCalculator = new DistanceCalculator(null);
    double distance = 3.5;
    double[] values = distanceCalculator.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 1;
    values = distanceCalculator.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 2;
    values = distanceCalculator.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 2.7;
    values = distanceCalculator.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 3;
    values = distanceCalculator.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);

    distance = 4;
    values = distanceCalculator.getVelocityAndHood(distance);
    System.out.println(
        "distance: " + distance + "  velocity: " + values[0] + "  hood: " + values[1]);
  }
}
