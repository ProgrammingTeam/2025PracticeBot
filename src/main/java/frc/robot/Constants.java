// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
// Alex was here 1/16/2025
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ElevatorConstants {
    public static final double kP = 0.18;
    public static final double kI = 0.0000;
    public static final double kD = 0;

    public static final double kMinOutput = 0;
    public static final double kMaxOutput = 0;
    public static final int kV = 0;
    public static final double maxVel = 0;
    public static final double maxAccel = 0;
    public static final double allowedErr = 0;

    private static final double conversionFactor = 0;
public static final double rotation = 69;
    public static enum ElevatorPositions {
      L1(11.33333333),
      L2(15.64),
      L3(18),
      L4(20.6),
      corolStation(37.7),
      travel(0),
      // net(4),
      // startAlgaeLow(3),
      // startAlgaeHigh(2),
      processor(1);

      public final double height;

      private ElevatorPositions(double height) {
        this.height = height;
      }
      // public static final double L1 = 18;
      // public static final double L2 = 31;
      // public static final double L3 = 47;
      // public static final double L4 = 72;
      // public static final double corolStation = 37;
      // public static final double travel = 0;
    }
  }
}