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

// The SequentialCommandGroup class of Score
public class Score extends SequentialCommandGroup {

  // Score constructor
  public Score(ElevatorSub m_ElvSub, FunnelSub SubFunnel, ElevatorPositions elevatorPosition) {
    // alex was here and so was Loki
    addCommands(new ParallelRaceGroup (new AutoElevatorCmd(m_ElvSub, elevatorPosition)));
    addCommands(new ParallelRaceGroup (new DispenserCommand(SubFunnel), new WaitCommand(4)));
    addCommands(new ParallelRaceGroup(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.travel)));
  }
}
