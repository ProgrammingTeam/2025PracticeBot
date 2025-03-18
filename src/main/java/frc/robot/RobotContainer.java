// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.AutoElevatorCmd;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ElevatorSub;

import frc.robot.commands.DriveCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.LimelightPositionCom;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSub;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;


import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DispenserCommand;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Score;
import frc.robot.commands.RotatorFwdCmd;
import frc.robot.commands.RotatorBwdCmd;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.AlgaeSub;
import frc.robot.subsystems.FunnelSub;
import frc.robot.subsystems.SwerveSub;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

// The class def of RobotContainer; contains subsystem defs and controllers
public class RobotContainer {
  SwerveDrive m_Swerve;
 private final AlgaeSub algae = new AlgaeSub();
 private final FunnelSub FunnelSubSystem = new FunnelSub();
  private final ElevatorSub m_ElvSub = new ElevatorSub();
  private final SwerveSub subSwerve;
//  private final LimelightSub m_LimelightSub;
  

  private final ElevatorCmd m_ElevatorCmd;
  private final DriveCmd driveCom;
  private final IntakeCommand inCom; 
  private final DispenserCommand disCom;
  private final RotatorFwdCmd fwdCom;
  private final RotatorBwdCmd bwdCom;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandJoystick leftJoystick = new CommandJoystick(OperatorConstants.LeftJoystickPort);
  private final CommandJoystick rightJoystick = new CommandJoystick(OperatorConstants.RightJoystickPort);
  private final SendableChooser<Command> autoChooser;

  // Constructor of RobotContainer; Contains subsystems, OI devices, and commands init
  public RobotContainer() {
    CanandEventLoop.getInstance();
    m_ElevatorCmd = new ElevatorCmd(m_ElvSub, m_driverController, rightJoystick);
    inCom = new IntakeCommand(FunnelSubSystem);
    disCom = new DispenserCommand(FunnelSubSystem);
    fwdCom = new RotatorFwdCmd(algae);
    bwdCom = new RotatorBwdCmd(algae);
    
    try {
      double maximumSpeed = 0.1;
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_Swerve = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("Failed");
      e.printStackTrace();
    }
    
    subSwerve = new SwerveSub(m_Swerve, m_ElvSub);
    //m_LimelightSub = new LimelightSub(subSwerve);
    driveCom = new DriveCmd(subSwerve, leftJoystick, rightJoystick);
    m_ElvSub.setDefaultCommand(m_ElevatorCmd);
    subSwerve.setDefaultCommand(driveCom);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //NamedCommands.registerCommand("Score L3", new ScoreL3(m_ElvSub, FunnelSubSystem));


    configureBindings();
  }

  // Method for configuring/init the bindings; Joysticks, Xbox, Keyboard, etc...
  private void configureBindings() {
    leftJoystick.button(7).onTrue(new InstantCommand(subSwerve::zeroGyro, subSwerve));

    // leftJoystick.button(3).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  true));
    // rightJoystick.button(4).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  false));

    leftJoystick.button(1).whileTrue(inCom);
    rightJoystick.button(1).whileTrue(disCom);
    m_driverController.x().onTrue(new InstantCommand(() -> {
     algae.arm.setSetpoint(0);
    }));
    m_driverController.y().onTrue(new InstantCommand(() -> {
      algae.arm.setSetpoint(2.67);
    }));

    // Right joystick left side
    // - - 4
    // 3 2 1
    rightJoystick.button(8).onTrue(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.L1));
    rightJoystick.button(9).onTrue(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.L2));
    rightJoystick.button(10).onTrue(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.L3));
    rightJoystick.button(7).onTrue(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.L4));

    // Right joystick right side
    // - - e
    // c - -
    rightJoystick.button(14).onTrue(new AutoElevatorCmd(m_ElvSub, ElevatorPositions.corolStation));
    rightJoystick.button(11).onTrue(new InstantCommand(m_ElvSub::resetEncoder));



    m_driverController.leftBumper().whileTrue(fwdCom);
    m_driverController.rightBumper().whileTrue(bwdCom);


    // Funnel button commands -- button linking
    // m_driverController.a().onTrue(inCom);
    // m_driverController.b().onTrue(disCom);

    // PID elevator commands -- button linking
    //  m_driverController.y().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L1));
    //  m_driverController.b().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L2));
    //  m_driverController.a().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L3));
    //  m_driverController.x().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L4));
    // m_driverController.leftBumper().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.corolStation));
    // m_driverController.rightBumper().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.travel));

// pid elevator commands
      //  m_driverController.y().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L1));
      //  m_driverController.b().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L2));
      //  m_driverController.a().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L3));
      //  m_driverController.x().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.L4));
      // m_driverController.leftBumper().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.corolStation));
      // m_driverController.rightBumper().and(m_driverController.pov(180).negate()).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.travel));

      //m_driverController.a().and(m_driverController.pov(180)).onTrue(new ElevatorMoveButton(m_ElevatorSub, ElevatorPositions.net));
      //m_driverController.a().and(m_driverController.pov(180)).onTrue(new ElevatorMoveButton(m_ElevatorSub, ElevatorPositions.startAlgaeLow));
      // m_driverController.a().and(m_driverController.pov(180)).onTrue(new ElevatorMoveButton(m_ElevatorSub, ElevatorPositions.startAlgaeHigh));
      // m_driverController.a().and(m_driverController.pov(180)).onTrue(new ElevatorMoveButton(m_ElvSub, ElevatorPositions.processor));
    // m_driverController.b().whileTrue();
  }

  // Alex was here 2/18/2025 and so was I Loki 3/18/2025

  // Defines the getAutonomousCommand to return a selected autonomous plan
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
