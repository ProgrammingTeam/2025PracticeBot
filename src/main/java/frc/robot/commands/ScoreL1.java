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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL1 extends SequentialCommandGroup {

  /** Creates a new scoreL1. */

  ScoreL1(ElevatorSub m_ElvSub, FunnelSub SubFunnel) {
    addCommands(
      new ParallelRaceGroup(
        new WaitCommand(3),
        new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L1)
      ),
      new ParallelRaceGroup(
        new WaitCommand(3),
        new DispenserCommand(SubFunnel) 
      ),
      new ParallelRaceGroup(
        new WaitCommand(3),
        new ElevatorMoveButton(m_ElvSub, ElevatorPositions.travel)
      )
    );
  }
  }


