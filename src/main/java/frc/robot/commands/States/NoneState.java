// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLEDs;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constStateMachine;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;

public class NoneState extends Command {
  StateMachine subStateMachine;
  Climber subClimber;
  Elevator subElevator;
  Intake subIntake;
  LEDs subLEDs;
  Shooter subShooter;
  Transfer subTransfer;

  /** Creates a new NoneState. */
  public NoneState(StateMachine subStateMachine, Climber subClimber, Elevator subElevator, Intake subIntake,
      LEDs subLEDs, Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterPositionGroup desiredShooterPosition = constStateMachine.TARGET_TO_PRESET_GROUP.get(TargetState.PREP_NONE);

    subStateMachine.setQueueState(TargetState.PREP_NONE);
    subStateMachine.setRobotState(RobotState.NONE);
    subIntake.setIntakeRollerSpeed(Units.Percent.zero());
    subTransfer.setGamePieceCollected(false);
    subClimber.setSafeToMoveClimber(false);
    subElevator.setDrainpipeSpeed(0);
    subTransfer.setFeederSpeed(0);
    subLEDs.clearAnimation();
    subLEDs.setLEDs(constLEDs.CLEAR_LEDS);
    subShooter.setDesiredVelocities(desiredShooterPosition.leftVelocity, desiredShooterPosition.rightVelocity);
    subShooter.setShootingNeutralOutput();

    if (subShooter.isSafeToMoveElevator()) {
      subShooter.setPivotNeutralOutput();
    } else {
      subShooter.setPivotPosition(constShooter.NONE_STATE_ANGLE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setPivotNeutralOutput();
    subElevator.setElevatorPosition(constElevator.BACKWARD_LIMIT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subShooter.isSafeToMoveElevator();
  }
}
