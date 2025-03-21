// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

// Class t=of SwerveSub
public class SwerveSub extends SubsystemBase {
  SwerveDrive swerveDrive;
  ElevatorSub m_ElvSub;

  // Constructor for SwerveSub
  public SwerveSub(SwerveDrive swerve, ElevatorSub elevatorSub) {
    swerveDrive = swerve;
    m_ElvSub = elevatorSub;
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    RobotConfig config;
    
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      config = null;
      e.printStackTrace();
    }

    // Pose builder configuration
    AutoBuilder.configure(
      swerveDrive::getPose, // Robot pose supplier
      swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> swerveDrive.drive(speeds),
      // {
      //   swerveDrive.drive(speeds);
      //   SmartDashboard.putNumber("swerve speeds",speeds.vxMetersPerSecond);
      // }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // This will flip the path being followed to the red side of the field.
              // PATH WILL BE MIRRORED TO THE OPPOSITE SIDE, ORIGIN WILL REMAIN ON BLUE SIDE
              // Gets the current alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // TODO: Possibly missing an operand?
    );
  }

  // Drive method but scaled
  public void driveScaled(double x, double y, double rot) {    
    // alex was here 2/18/2025
    // swerveDrive.drive(new Translation2d(y, x), rot, true, false);
    this.driveUnscaled(x * m_ElvSub.elevatorDriveSpeedMultiplier, 
                       y * m_ElvSub.elevatorDriveSpeedMultiplier, 
                       rot * m_ElvSub.elevatorDriveSpeedMultiplier);
  }

  // Drive method but scaled
  public void driveUnscaled(double x, double y, double rot) {
    // swerveDrive.drive(new Translation2d(y, x), rot, true, false);
    swerveDrive.driveFieldOriented(new ChassisSpeeds(x, y, rot));
  }
  // Resets the gyro
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber( "Max chassis Velocity", swerveDrive.getMaximumChassisVelocity());
  }
}
