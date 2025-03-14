// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSub;

public class DriveCmd extends Command {
  private final SwerveSub swerveSub;
  private final CommandJoystick RJoystick;
  private final CommandJoystick LJoystick;

  public DriveCmd(SwerveSub swerveSubsystem, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    swerveSub = swerveSubsystem;
    RJoystick = rightJoystick;
    LJoystick = leftJoystick;

    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSub.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.drive(
        (Math.pow(LJoystick.getRawAxis(Constants.joyY), 1) * Constants.speedMultiplier) * (LJoystick.getRawAxis(Constants.joySilder)-1) * -1/2,
        (Math.pow(LJoystick.getRawAxis(Constants.joyX), 1) * Constants.speedMultiplier) * (LJoystick.getRawAxis(Constants.joySilder)-1) * -1/2,
        (Math.pow(RJoystick.getRawAxis(Constants.joyX), 1) * Constants.speedMultiplier) * (RJoystick.getRawAxis(Constants.joySilder)-1) * -1/2
    );
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
