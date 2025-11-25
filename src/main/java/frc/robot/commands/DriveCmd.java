// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.SwerveSub;

public class DriveCmd extends Command {
  private final SwerveSub swerveSub;
  private final CommandXboxController m_XBox;

  public DriveCmd(SwerveSub swerveSubsystem, CommandXboxController xbox) {
    swerveSub = swerveSubsystem;
    m_XBox = xbox;
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSub.driveUnscaled(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.driveUnscaled(
        MathUtil.applyDeadband(m_XBox.getRawAxis(Constants.joyY), 0.05) * Constants.speedMultiplier,
        MathUtil.applyDeadband(m_XBox.getRawAxis(Constants.joyX), 0.05) * Constants.speedMultiplier,
        m_XBox.getRawAxis(4)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.driveUnscaled(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
