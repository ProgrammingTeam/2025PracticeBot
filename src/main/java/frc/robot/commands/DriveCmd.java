// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.SwerveSub;

public class DriveCmd extends Command {
  private final SwerveSub swerveSub;
  private final CommandJoystick RJoystick;
  private final CommandJoystick LJoystick;

  public DriveCmd(SwerveSub swerveSubsystem, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    RJoystick = rightJoystick;
    LJoystick = leftJoystick;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSub.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // X = 0, Y = 1, Z = 2, Slider = 3
    swerveSub.drive(
        (LJoystick.getRawAxis(0) * Constants.speedMultiplier) * LJoystick.getRawAxis(3),
        (LJoystick.getRawAxis(1) * Constants.speedMultiplier) * LJoystick.getRawAxis(3),
        (RJoystick.getRawAxis(0) * Constants.speedMultiplier) * RJoystick.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
