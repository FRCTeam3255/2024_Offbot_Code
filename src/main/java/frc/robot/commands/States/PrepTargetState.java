// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constStateMachine;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

public class PrepTargetState extends Command {
  StateMachine subStateMachine;
  Shooter subShooter;
  Elevator subElevator;
  TargetState desiredTargetState;

  Measure<Angle> desiredPivotAngle;
  Measure<Velocity<Angle>> desiredLeftVelocity, desiredRightVelocity;
  ShooterPositionGroup desiredShooterPosition;
  boolean elevatorWasUp;

  /** Creates a new PrepTargetState. */
  public PrepTargetState(Elevator subElevator, StateMachine subStateMachine, Shooter subShooter,
      TargetState desiredTargetState) {
    this.subStateMachine = subStateMachine;
    this.subShooter = subShooter;
    this.subElevator = subElevator;
    this.desiredTargetState = desiredTargetState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState desiredRobotState = constStateMachine.TARGET_TO_ROBOT_STATE.get(desiredTargetState);
    RobotState currentRobotState = subStateMachine.getRobotState();

    if (currentRobotState.equals(RobotState.STORE_FEEDER) || subStateMachine.isCurrentStateTargetState()) {
      subStateMachine.setRobotState(desiredRobotState);
    }

    desiredShooterPosition = constStateMachine.TARGET_TO_PRESET_GROUP.get(desiredTargetState);

    elevatorWasUp = subElevator.isSafeToMoveShooterAboveLimit();

    if (elevatorWasUp) {
      subShooter.setDesiredPosition(desiredShooterPosition);
    } else {
      subElevator.setElevatorPosition(desiredShooterPosition.elevatorPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setDesiredPosition(desiredShooterPosition);
    subElevator.setElevatorPosition(desiredShooterPosition.elevatorPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorWasUp) {
      return subShooter.isShooterAtPosition(desiredShooterPosition.shooterAngle);
    } else {
      return subElevator.isElevatorAtPosition(desiredShooterPosition.elevatorPosition);
    }
  }
}
