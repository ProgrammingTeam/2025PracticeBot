// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSub;

// Class of cmd LimelightPositionCom
public class LimelightPositionCom extends Command {
  private final LimelightSub m_LimelightSub;
  private final SwerveSub m_Swerve;
  private final boolean LeftOffset;
  boolean toPosition;
  
  // Constructor of the LimelightPosCom class
  public LimelightPositionCom(LimelightSub LimeSub, SwerveSub SwerveSub, boolean leftDirectionalOffset) {
    m_LimelightSub = LimeSub;
    m_Swerve = SwerveSub;
    LeftOffset = leftDirectionalOffset;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LimelightSub);
    addRequirements(m_Swerve);
  }

  // Called when initially scheduled
  @Override
  public void initialize() {
    toPosition = false;
    m_Swerve.driveScaled(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LeftOffset) {
      if(m_LimelightSub.distenceFromTarget > Constants.LimelightConstants.LeftPositionOffset + 1) {
        m_Swerve.driveScaled(0.2, 0, 0);
      } else if(m_LimelightSub.distenceFromTarget < Constants.LimelightConstants.LeftPositionOffset - 1) {
        m_Swerve.driveScaled(-0.2, 0, 0);
      } else {
        m_Swerve.driveScaled(0, 0, 0);
        toPosition = true;
      }
    } else {
      if(m_LimelightSub.distenceFromTarget > Constants.LimelightConstants.RightPositionOffset + 1) {
        m_Swerve.driveScaled(0.2, 0, 0);
      } else if(m_LimelightSub.distenceFromTarget < Constants.LimelightConstants.RightPositionOffset - 1) {
        m_Swerve.driveScaled(-0.2, 0, 0);
      } else {
        m_Swerve.driveScaled(0, 0, 0);
        toPosition = true;
      }
    }
  }
  
  // Called when interrupted -- Add a safety function 
  @Override
  public void end(boolean interrupted) {}

  // Return the position to when the cmd ends
  @Override
  public boolean isFinished() {
    return toPosition;
  }
}
