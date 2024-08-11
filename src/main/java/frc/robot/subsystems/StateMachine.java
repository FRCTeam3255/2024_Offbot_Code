// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.RobotState;
import frc.robot.RobotContainer.TargetState;
import frc.robot.commands.States.Eject;
import frc.robot.commands.States.IntakeFloor;
import frc.robot.commands.States.PrepShuffle;
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

  public Command tryState(RobotState desiredState, StateMachine subStateMachine, Intake subIntake,
      Transfer subTransfer, Shooter subShooter) {
    switch (desiredState) {
      case INTAKING:
        switch (currentState) {
          case NONE:
          case SHOOTING:
            return new IntakeFloor(subStateMachine, subIntake, subTransfer);
        }

      case STORE_FEEDER:
        switch (currentState) {
          case INTAKING:
            return new StoreFeeder(subStateMachine, subIntake, subTransfer);
        }

      case EJECTING:
        switch (currentState) {
          case NONE:
          case PREP_NONE:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
            return new Eject(subIntake, subTransfer, subStateMachine);
        }

      case PREP_SHUFFLE:
        switch (currentState) {
          case STORE_FEEDER:
            return new PrepShuffle(subStateMachine, subShooter);
        }

      case PREP_SPEAKER:
        switch (currentState) {
          case STORE_FEEDER:
            return new PrepShuffle(subStateMachine, subShooter);
        }

      default:
        return new StoreFeeder(subStateMachine, subIntake, subTransfer); // placeholder for now
      // TODO: make a command when tryState is invalid (flashing LEDs?)
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());

  }
}
