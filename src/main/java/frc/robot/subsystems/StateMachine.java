// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.UnPrepAmp;
import frc.robot.commands.States.Ejecting;
import frc.robot.commands.States.Intaking;
import frc.robot.commands.States.NoneState;
import frc.robot.commands.States.PrepAmp;
import frc.robot.commands.States.PrepShuffle;
import frc.robot.commands.States.PrepSpeaker;
import frc.robot.commands.States.Shooting;
import frc.robot.commands.States.StoreFeeder;

public class StateMachine extends SubsystemBase {
  public static RobotState currentState;
  public static TargetState currentTargetState;

  /** Creates a new StateMachine. */
  public StateMachine() {
    currentState = RobotState.NONE;
    currentTargetState = TargetState.NONE;
  }

  public void setRobotState(RobotState robotState) {
    currentState = robotState;
  }

  public void setTargetState(TargetState targetState) {
    currentTargetState = targetState;
  }

  public RobotState getRobotState() {
    return currentState;
  }

  public TargetState getTargetState() {
    return currentTargetState;
  }

  /**
   * Determines which command to run for a desired state depending on if our
   * current state.
   * 
   * @see <a
   *      href=https://www.tldraw.com/ro/DX06u039erL_iV6q0ARSn?d=v-1103.-1504.5212.2506.page>
   *      Our State Machine Diagram
   *      </a>
   * @param desiredState The state you would like to go to, which may not be
   *                     possible from your current state
   * @return The Command to run for that desired state
   */
  public Command tryState(RobotState desiredState, StateMachine subStateMachine, Elevator subElevator, Intake subIntake,
      Transfer subTransfer, Shooter subShooter) {
    switch (desiredState) {
      case NONE:
        switch (currentState) {
          case INTAKING:
          case EJECTING:
          case SHOOTING:
            return new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer);
        }
        break;

      case INTAKING:
        switch (currentState) {
          case NONE:
          case SHOOTING:
            return new Intaking(subStateMachine, subIntake, subShooter, subTransfer);
        }
        break;

      case STORE_FEEDER:
        switch (currentState) {
          case INTAKING:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
            return new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter);
          case PREP_AMP:
            return new UnPrepAmp(subStateMachine, subElevator, subShooter, subTransfer)
                .andThen(new Intaking(subStateMachine, subIntake, subShooter, subTransfer))
                .andThen(new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }
        break;

      case EJECTING:
        switch (currentState) {
          case NONE:
          case INTAKING:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
            return new Ejecting(subStateMachine, subIntake, subTransfer);
          case PREP_AMP:
            return new UnPrepAmp(subStateMachine, subElevator, subShooter, subTransfer)
                .andThen(new Intaking(subStateMachine, subIntake, subShooter, subTransfer))
                .andThen(new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter))
                .andThen(new Ejecting(subStateMachine, subIntake, subTransfer))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }
        break;

      case PREP_SHUFFLE:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SPEAKER:
            return new PrepShuffle(subStateMachine, subShooter);
          case PREP_AMP:
            return new UnPrepAmp(subStateMachine, subElevator, subShooter, subTransfer)
                .andThen(new Intaking(subStateMachine, subIntake, subShooter, subTransfer))
                .andThen(new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter))
                .andThen(new PrepShuffle(subStateMachine, subShooter))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }
        break;

      case PREP_SPEAKER:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
            return new PrepSpeaker(subStateMachine, subShooter);
          case PREP_AMP:
            return new UnPrepAmp(subStateMachine, subElevator, subShooter, subTransfer)
                .andThen(new Intaking(subStateMachine, subIntake, subShooter, subTransfer))
                .andThen(new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter))
                .andThen(new PrepSpeaker(subStateMachine, subShooter))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }
        break;

      case PREP_AMP:
        switch (currentState) {
          case STORE_FEEDER:
          case PREP_SPEAKER:
          case PREP_SHUFFLE:
            return new PrepAmp(subStateMachine, subElevator, subShooter, subTransfer);
        }
        break;

      case SHOOTING:
        switch (currentState) {
          case PREP_SPEAKER:
          case PREP_SHUFFLE:
          case PREP_AMP:
          case SHOOTING:
            return new Shooting(subStateMachine, subElevator, subShooter, subTransfer);
        }
        break;
    }
    return Commands.print("ITS SO OVER D: Invalid State Provided :3");
  }

  public Command tryTargetState(StateMachine subStateMachine, Intake subIntake,
      Shooter subShooter, Transfer subTransfer) {
    switch (currentTargetState) {
      case PREP_SHUFFLE:
        return new PrepShuffle(subStateMachine, subShooter);
      case PREP_SPEAKER:
        return new PrepSpeaker(subStateMachine, subShooter);
      default:
        return new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter);
    }
  }

  public static enum RobotState {
    NONE,
    INTAKING,
    STORE_FEEDER,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_AMP,
    CLIMBING,
    SHOOTING,
    EJECTING
  }

  public static enum TargetState {
    NONE,
    PREP_SHUFFLE,
    PREP_SPEAKER,
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());

  }
}
