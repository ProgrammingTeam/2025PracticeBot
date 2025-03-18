// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSub;

public class limelightPositionCom extends Command {
  private final LimelightSub m_LimelightSub;
  private final SwerveSub m_Swerve;
  private final boolean LeftOffset;
  boolean toPosition;

  // Constructor for LimelightPosition Command
  public limelightPositionCom(LimelightSub LimeSub, SwerveSub SwerveSub, boolean leftDirectionalOffset) {
    m_LimelightSub = LimeSub;
    m_Swerve = SwerveSub;
    LeftOffset = leftDirectionalOffset;
    
    addRequirements(m_LimelightSub);
    addRequirements(m_Swerve);
  }

  // Method that the robot calls when cmd is init
  @Override
  public void initialize() {
    toPosition = false;
    m_Swerve.drive(0, 0, 0);
  }

  // Method that runs when execute is scheduled -- main position logic is here
  @Override
  public void execute() {
    if (LeftOffset) {
      if(m_LimelightSub.distenceFromTarget > Constants.LimelightConstants.LeftPositionOffset + 1) {
        m_Swerve.drive(0.2, 0, 0);
      } else if(m_LimelightSub.distenceFromTarget < Constants.LimelightConstants.LeftPositionOffset - 1) {
        m_Swerve.drive(-0.2, 0, 0);
      } else {
        m_Swerve.drive(0, 0, 0);
        toPosition = true;
      }
    } else {
      if(m_LimelightSub.distenceFromTarget > Constants.LimelightConstants.RightPositionOffset + 1) {
        m_Swerve.drive(0.2, 0, 0);
      } else if(m_LimelightSub.distenceFromTarget < Constants.LimelightConstants.RightPositionOffset - 1) {
        m_Swerve.drive(-0.2, 0, 0);
      } else {
        m_Swerve.drive(0, 0, 0);
        toPosition = true;
      }
    }
  }
  
  // Do nothing if the cmd is interrupted
  // TODO: Add safety features if this cmd is stopped, like stop calc.
  @Override
  public void end(boolean interrupted) {}

  // Return the position when the cmd ends
  @Override
  public boolean isFinished() {
    return toPosition;
  }
}
