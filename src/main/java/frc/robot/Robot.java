// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.preferences.SN_Preferences;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean hasAutonomousRun = false;
  private boolean bothSubsystemsZeroed = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // Set out log file to be in its own folder
    if (Robot.isSimulation()) {
      DataLogManager.start("src/main");
    } else {
      DataLogManager.start();
    }
    // Log data that is being put to shuffleboard
    DataLogManager.logNetworkTables(true);
    // Log the DS data and joysticks
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    DriverStation.silenceJoystickConnectionWarning(Constants.constControllers.SILENCE_JOYSTICK_WARNINGS);

    SN_Preferences.useDefaults();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotContainer.AddVisionMeasurement().schedule();
    RobotContainer.logPDHValues();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.subStateMachine.setRobotState(RobotState.NONE);
    m_robotContainer.subStateMachine.setTargetState(TargetState.PREP_NONE);
    m_robotContainer.setMegaTag2(false);

    Shooter.hasZeroed = false;
    Elevator.hasZeroed = false;

    if (!hasAutonomousRun) {
      RobotContainer.checkForManualZeroing().schedule();
    }
    m_robotContainer.setDisabledLEDs();
  }

  @Override
  public void disabledPeriodic() {
    constField.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", constField.ALLIANCE.toString());
    m_robotContainer.setZeroedLEDs();
  }

  @Override
  public void disabledExit() {
    m_robotContainer.clearLEDs();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMegaTag2(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    bothSubsystemsZeroed = Shooter.hasZeroed && Elevator.hasZeroed;

    if (bothSubsystemsZeroed && m_autonomousCommand != null) {
      Commands.deferredProxy(() -> m_autonomousCommand).schedule();
    } else if (m_autonomousCommand != null) {
      RobotContainer.zeroSubsystems().andThen(Commands.deferredProxy(() -> m_autonomousCommand)).schedule();
    } else {
      RobotContainer.zeroSubsystems().schedule();
    }

    hasAutonomousRun = true;
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    bothSubsystemsZeroed = Shooter.hasZeroed && Elevator.hasZeroed;
    m_robotContainer.setMegaTag2(true);

    RobotContainer.checkForManualZeroing().cancel();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!hasAutonomousRun) {
      RobotContainer.zeroSubsystems().schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
