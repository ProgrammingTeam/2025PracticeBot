// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Constants;

public class DriveCmd extends Command {
  private final SwerveSub swerveSub;
  private final CommandJoystick RJoystick;
  private final CommandJoystick LJoystick;
  private boolean isSlow;

  public DriveCmd(SwerveSub swerveSubsystem, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    RJoystick = rightJoystick;
    LJoystick = leftJoystick;

    addRequirements(getRequirements());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSub.setMotors(0, 0);
    isSlow = false;
    //TODO: CONFIGURE BUTTON PRESS FOR SLOW MODE 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: CHECK JOYSTICK AXIS AND SET THEM TO A CONSTANT FOR BETTER READABILITY
    if (isSlow == false) {
      // 1 is added to axis 2 to increments the value, not reduce the value.
      diffDrive.curvatureDrive(LJoystick.getRawAxis(1) * (LJoystick.getRawAxis(2) + 1) / 2,
          RJoystick.getRawAxis(1), true);
    } else if (isSlow == true) {
      diffDrive.curvatureDrive((LJoystick.getRawAxis(1) * (LJoystick.getRawAxis(2) + 1) / 2) / 2,
          (RJoystick.getRawAxis(1)) / 2, true);
    } else {
      diffDrive.curvatureDrive(LJoystick.getRawAxis(1) * (LJoystick.getRawAxis(2) + 1) / 2,
          RJoystick.getRawAxis(1), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
