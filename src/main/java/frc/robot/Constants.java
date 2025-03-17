// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Alex was here 1/16/2025
// Class that is the parent of other constant classes; has general defs such as axis defs
public final class Constants {
    public static final double speedMultiplier = 5;

    public static final int joyX = 0;
    public static final int joyY = 1;
    public static final int joyZ = 2;
    public static final int joySilder = 3;

    // Constants for the elevator -- speed, accel, etc...
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

    // Constants for the port of the joysticks
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int LeftJoystickPort = 1;
        public static final int RightJoystickPort = 2;
    }

    // Constants for the limelight e.g. the target heights and the position of the limelight
    public static class LimelightConstants {
        public static final double[] targetHeights = {0, 58.5, 58.5, 51, 74.25, 74.25, 12, 66, 66, 12, 12, 12,
                                                         58.5, 58.5, 74.25, 74.2, 51.5, 12, 12, 12 ,12 ,12 ,12};
        public static final double limelightHeight = 44.5;
        public static final double angleOffset = 0;
        public static final double LeftPositionOffset = 0;
        public static final int RightPositionOffset = 0;
    }

    // Contains gear ratios and motor rots
    public static class AlgeaConstants {
        public static final double Kp = 0.3; //Motor rotations per rotation of the arm
        public static final double armGearRatio = 1/4;
        public static final double Kp = 12;
    }
}
