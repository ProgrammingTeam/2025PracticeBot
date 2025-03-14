// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    // General Constants -- joystick numerals, speed multipliers, gear ratios, etc...
    public static final double speedMultiplier = 3.81;

    public static final int joyX = 0;
    public static final int joyY = 1;
    public static final int joyZ = 2;
    public static final int joySilder = 3;

    public static class OperatorConstants {
        // Operator Constants -- Joystick ports
        public static final int kDriverControllerPort = 0;
        public static final int LeftJoystickPort = 1;
        public static final int RightJoystickPort = 2;
    }

    public static class LimelightConstants {
        // Constants directly concentrated to limelight 
        public static final double[] targetHeights = {0, 58.5, 58.5, 51, 74.25, 74.25, 12, 66, 66, 12, 12, 12,
                                                         58.5, 58.5, 74.25, 74.2, 51.5, 12, 12, 12 ,12 ,12 ,12};
        public static final double limelightHeight = 44.5;
        public static final double angleOffset = 0;
        public static final double LeftPositionOffset = 0;
        public static final int RightPositionOffset = 0;
    }

    public static class CANIds {
        // Motor CAN ID defs
        // TODO: MAKE BETTER MOTOR NAMING CONVENTIONS FOR GENERAL READABILITY
        public static final int someMotorA = 31;
        public static final int someMotorB = 32;
        public static final int someMotorC = 33;
        public static final int someMotorD = 34;
    }

    public static class AlgeaConstants {
        // Contains ratios
        // TODO: FIX NAMING CONVENTION FOR CLASS DEF -- USE RED HAT TOOLS TO MODIFY ACROSS ALL FILES
        public static final double armGearRatio = 1/4;
    }
}
