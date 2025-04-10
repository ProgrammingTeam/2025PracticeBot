// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FunnelSub;

// Class for the ScoreL1 Sequential Command Group
public class ScoreL1 extends SequentialCommandGroup {
  
  // Constructor of ScoreL1 SequentialCommandGroup
  ScoreL1(ElevatorSub m_ElvSub, FunnelSub SubFunnel) {
    // alex was here
    addCommands(new ParallelRaceGroup(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.L1)));
    addCommands(new ParallelRaceGroup(new DispenserCommand(SubFunnel), new WaitCommand(4)));
    addCommands(new ParallelRaceGroup(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.travel)));
  }
}


