// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

// The subsystem class
public class SwerveSub extends SubsystemBase {
SwerveDrive swerveDrive;

  // The swerveSub constructor to init
  public SwerveSub(SwerveDrive swerve) {
    swerveDrive = swerve;
    RobotConfig config;
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }
    
    AutoBuilder.configure(
      swerveDrive::getPose, // Robot pose supplier
      swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> swerveDrive.drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            }
    );
  }

  public void drive(double x, double y, double rot) {
    // swerveDrive.drive(new Translation2d(y, x), rot, true, false);
    // Controls the motors via the drive method called externally
    swerveDrive.driveFieldOriented(new ChassisSpeeds(x, y, rot));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  @Override
  public void periodic() {
   // Called periodically -- every scheduler run
  }
}
