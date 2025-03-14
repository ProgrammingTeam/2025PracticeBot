// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.limelightPositionCom;
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
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import frc.robot.subsystems.FunnelSub;
import frc.robot.subsystems.SwerveSub;
import swervelib.parser.SwerveParser;

// Main file that constructs the robot, and handles configuration.
public class RobotContainer {
  // Subsystems and commands defined in this class
  
  SwerveDrive m_Swerve;

  private final FunnelSub FunnelSubSystem = new FunnelSub();

  private final SwerveSub subSwerve;
  private final LimelightSub m_LimelightSub = new LimelightSub();
  
  private final SendableChooser<Command> autoChooser;

 
  private final DriveCmd driveCom;
  private final IntakeCommand inCom; 
  private final DispenserCommand disCom;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandJoystick leftJoystick = new CommandJoystick(OperatorConstants.LeftJoystickPort);
  private final CommandJoystick rightJoystick = new CommandJoystick(OperatorConstants.RightJoystickPort);

  // Container -- Constructor
  public RobotContainer() {
    // Constructs event loop, cmds, and subs
    CanandEventLoop.getInstance();

    inCom = new IntakeCommand(FunnelSubSystem);
    disCom = new DispenserCommand(FunnelSubSystem);

    try {
      double maximumSpeed = Units.feetToMeters(4.5);
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_Swerve = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("Failed");
      e.printStackTrace();
    }
    
    subSwerve = new SwerveSub(m_Swerve);
    driveCom = new DriveCmd(subSwerve, leftJoystick, rightJoystick);
    
    subSwerve.setDefaultCommand(driveCom); 
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  // Configures/Constructs the binds for external control -- joystick
  private void configureBindings() {
    leftJoystick.button(7).onTrue(new InstantCommand(subSwerve::zeroGyro, subSwerve));

    leftJoystick.button(3).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  true));
    rightJoystick.button(4).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  false));

    m_driverController.a().whileTrue(inCom);
    m_driverController.b().whileTrue(disCom);

    // Example use of xbox controller
    // m_driverController.b().whileTrue();
  }

  // Command to init the autonomous cmd
  public Command getAutonomousCommand() {
    // Returns the auto cmd to the caller in Main.java
    return new PathPlannerAuto("Example Auto");
    // TODO: ADD A AUTO CMD
  }
}
