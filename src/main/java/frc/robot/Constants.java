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
  public static final double speedMultiplier = 5;

  public static final int joyX = 0;
  public static final int joyY = 1;
  public static final int joyZ = 2;
  public static final int joySilder = 3;

  public static class CANBus {

    //drive 
    public static final int FLAngle = 1;
    public static final int FLDrive = 2;
    public static final int BLAngle = 3;
    public static final int BLDrive = 4;
    public static final int FRAngle = 5;
    public static final int FRDrive = 6;
    public static final int BRAngle = 7;
    public static final int BRDrive = 8;
    public static final int gyro = 9;

    //elevator
    public static final int lElevator = 21;
    public static final int rElevator = 22;   

    //coral manipulator
    public static final int coralIntake = 33;
    public static final int lCoralShooter = 31;
    public static final int rCoralShooter = 32;

    //algae grabber
    public static final int algaeArm = 41;
    public static final int algaeRotator = 42;
  }

  public static class ElevatorConstants {
    public static final double kP = 0.18;
    public static final double kI = 0.0000;
    public static final double kD = 0;

    public static final double mountingHeight = 9.7; //inch
    public static final double rotationsPerInch = 1.927; //full revolutions of motor per inch elevator raised

    public static enum ElevatorPositions {
      L1(24),
      L2(35),
      L3(47),
      L4(77),
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


    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int LeftJoystickPort = 1;
        public static final int RightJoystickPort = 2;
    }

    public static class LimelightConstants {

        public static final double[] targetHeights = {0, 58.5, 58.5, 51, 74.25, 74.25, 12, 66, 66, 12, 12, 12,
                                                         58.5, 58.5, 74.25, 74.2, 51.5, 12, 12, 12, 12, 12, 12};
        public static final double limelightHeight = 44.5;
        public static final double angleOffset = 0;
        public static final double LeftPositionOffset = 0;
        public static final int RightPositionOffset = 0;

    }

    public static class AlgeaConstants {
        public static final double Kp = 0.6; 
        
        //Motor rotations per rotation of the arm
        public static final double armGearRatio = 0.2;
    }

}
