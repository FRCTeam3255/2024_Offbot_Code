// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.Constants.constLEDs;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;

public class StoreFeeder extends Command {
  StateMachine subStateMachine;
  Intake subIntake;
  LEDs subLEDs;
  Transfer subTransfer;
  Shooter subShooter;

  /** Creates a new StoreTransfer. */
  public StoreFeeder(StateMachine subStateMachine, Intake subIntake, LEDs subLEDs, Transfer subTransfer,
      Shooter subShooter) {
    this.subStateMachine = subStateMachine;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.STORE_FEEDER);
    subLEDs.setLEDAnimation(constLEDs.STORE_FEEDER_COLOR, 0);
    // Don't stop the feeders if we're intaking from source because it'll stop too
    // soon
    if (subStateMachine.getRobotState() != RobotState.INTAKE_SOURCE) {
      subIntake.setIntakeRollerSpeed(Units.Percent.zero());
      subTransfer.setFeederSpeed(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
