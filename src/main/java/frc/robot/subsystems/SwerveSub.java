// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSub extends SubsystemBase {
SwerveDrive swerveDrive;

  /** Creates a new SwerveSub. */
  public SwerveSub(SwerveDrive swerve) {
    swerveDrive = swerve;
  }

  public void drive(double x, double y, double rot) {
    // swerveDrive.drive(new Translation2d(y, x), rot, true, false);
    swerveDrive.driveFieldOriented(new ChassisSpeeds(x, y, rot));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  @Override
  public void periodic() {
    // This method will be called on  123rh ce per scheduler run
  }
}
