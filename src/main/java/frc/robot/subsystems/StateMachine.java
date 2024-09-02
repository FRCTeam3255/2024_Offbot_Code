// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public Command tryState(RobotState desiredState, StateMachine subStateMachine, Elevator subElevator, Intake subIntake,
      Transfer subTransfer, Shooter subShooter) {
    switch (desiredState) {
      case INTAKING:
        switch (currentState) {
          case NONE:
          case SHOOTING:
            return new Intaking(subStateMachine, subIntake, subTransfer);
        }
        break;

      case STORE_FEEDER:
        switch (currentState) {
          case INTAKING:
            return new StoreFeeder(subStateMachine, subIntake, subTransfer);
        }
        break;

      case EJECTING:
        switch (currentState) {
          case NONE:
          case INTAKING:
          case STORE_FEEDER:
          case PREP_NONE:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
            return new Ejecting(subStateMachine, subIntake, subTransfer);
        }
        break;

      case PREP_SHUFFLE:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SPEAKER:
          case PREP_NONE:
            return new PrepShuffle(subStateMachine, subShooter);
        }
        break;

      case PREP_SPEAKER:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_NONE:
            return new PrepSpeaker(subStateMachine, subShooter);
        }
        break;

      case PREP_AMP:
        switch (currentState) {
          case STORE_FEEDER:
          case PREP_SPEAKER:
          case PREP_SHUFFLE:
          case PREP_NONE:
            return new PrepAmp(subStateMachine, subElevator, subShooter, subTransfer);
        }
        break;

      case SHOOTING:
        switch (currentState) {
          case PREP_NONE:
          case PREP_SPEAKER:
          case PREP_SHUFFLE:
          case PREP_AMP:
          case SHOOTING:
            return new Shooting(subStateMachine, subElevator, subShooter, subTransfer);
        }
    }
    // TODO: replace NoneState with a default command when previous states were
    // invalid (flashing LEDs?)
    return new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer);
  }

  public static enum RobotState {
    NONE,
    INTAKING,
    STORE_FEEDER,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_AMP,
    PREP_NONE,
    CLIMBING,
    SHOOTING,
    EJECTING
  }

  public static enum TargetState {
    NONE,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_NONE
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());

  }
}