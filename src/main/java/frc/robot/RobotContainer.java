// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.Autos;

import frc.robot.commands.ElevatorMoveButton;
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

import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.ScoreL3;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.AlgaeSub;
import frc.robot.subsystems.FunnelSub;
import frc.robot.subsystems.SwerveSub;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SwerveDrive m_Swerve;

  private final FunnelSub FunnelSubSystem = new FunnelSub();
  private final ElevatorSub m_ElvSub = new ElevatorSub();
  private final SwerveSub subSwerve;
  private final LimelightSub m_LimelightSub;
  private final AlgaeSub algae = new AlgaeSub();

  private final ElevatorCmd m_ElevatorCmd;
  private final DriveCmd driveCom;
  private final IntakeCommand inCom; 
  private final DispenserCommand disCom;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandJoystick leftJoystick = new CommandJoystick(OperatorConstants.LeftJoystickPort);
  private final CommandJoystick rightJoystick = new CommandJoystick(OperatorConstants.RightJoystickPort);
  private final SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CanandEventLoop.getInstance();
    m_ElevatorCmd = new ElevatorCmd(m_ElvSub, m_driverController, rightJoystick);
    inCom = new IntakeCommand(FunnelSubSystem);
    disCom = new DispenserCommand(FunnelSubSystem);

    try {
      double maximumSpeed = 0.1;
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_Swerve = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("Failed");
      e.printStackTrace();
    }
    subSwerve = new SwerveSub(m_Swerve, m_ElvSub);
    m_LimelightSub = new LimelightSub(subSwerve);
    driveCom = new DriveCmd(subSwerve, leftJoystick, rightJoystick);
    m_ElvSub.setDefaultCommand(m_ElevatorCmd);
    // subSwerve.setDefaultCommand(driveCom);
   autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("Score L3", new ScoreL3(m_ElvSub, FunnelSubSystem));


    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // leftJoystick.button(7).onTrue(new InstantCommand(subSwerve::zeroGyro, subSwerve));

    // leftJoystick.button(3).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  true));
    // rightJoystick.button(4).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  false));

    // m_driverController.a().whileTrue(inCom);
    // m_driverController.b().whileTrue(disCom);
    // m_driverController.x().onTrue(new InstantCommand(() -> {
    //  algae.arm.setSetpoint(0);
    // }));
    // m_driverController.y().onTrue(new InstantCommand(() -> {
      // algae.arm.setSetpoint(7.5);
    // }));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //funnel Comands
      // m_driverController.a().onTrue(inCom);
      // m_driverController.b().onTrue(disCom);

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * Alex was here 2/18/2025
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
