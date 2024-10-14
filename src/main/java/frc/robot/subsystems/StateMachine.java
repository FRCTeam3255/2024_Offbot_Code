// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constClimber;
import frc.robot.Constants.constStateMachine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.States.Climbing;
import frc.robot.commands.States.Ejecting;
import frc.robot.commands.States.IntakeSource;
import frc.robot.commands.States.Intaking;
import frc.robot.commands.States.NoneState;
import frc.robot.commands.States.PrepTargetState;
import frc.robot.commands.States.PrepVision;
import frc.robot.commands.States.Shooting;
import frc.robot.commands.States.StoreFeeder;

public class StateMachine extends SubsystemBase {
  public static RobotState currentState;
  public static TargetState currentTargetState;

  /** Creates a new StateMachine. */
  public StateMachine() {
    currentState = RobotState.NONE;
    currentTargetState = TargetState.PREP_NONE;
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
  public Command tryState(RobotState desiredState, StateMachine subStateMachine, Climber subClimber,
      Drivetrain subDrivetrain,
      Elevator subElevator, Intake subIntake,
      Transfer subTransfer, Shooter subShooter) {

    // TODO: Write this functionality in a later pr
    if (isGivenStateTargetState(desiredState)) {
      // functionality is always the same for a target state
    }

    switch (desiredState) {
      case NONE:
        switch (currentState) {
          case INTAKE_SOURCE:
          case INTAKING:
          case EJECTING:
          case SHOOTING:
          case NONE:
            return new NoneState(subStateMachine, subClimber, subElevator, subIntake, subShooter, subTransfer);
        }
        break;

      case INTAKING:
        switch (currentState) {
          case NONE:
          case INTAKE_SOURCE:
          case SHOOTING:
            return new Intaking(subStateMachine, subIntake, subShooter, subTransfer);
        }
        break;

      case INTAKE_SOURCE:
        switch (currentState) {
          case NONE:
          case INTAKE_SOURCE:
            return new IntakeSource(subStateMachine, subShooter, subTransfer);
        }

      case STORE_FEEDER:
        switch (currentState) {
          case STORE_FEEDER:
          case INTAKING:
          case INTAKE_SOURCE:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter);
        }
        break;

      case CLIMBING:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new Climbing(subClimber, subElevator, subStateMachine, subShooter, subTransfer);
        }

        break;
      case EJECTING:
        switch (currentState) {
          case NONE:
          case INTAKING:
          case INTAKE_SOURCE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new Ejecting(subStateMachine, subIntake, subElevator, subTransfer);
        }
        break;

      case SHOOTING:
        switch (currentState) {
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_SHUFFLE:
          case PREP_AMP:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_SUB_BACKWARDS:
          case PREP_NONE:
          case CLIMBING:
          case SHOOTING:
            return new Shooting(subStateMachine, subElevator, subShooter, subTransfer);
        }
        break;

      // -- PREPS --
      case PREP_SHUFFLE:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_SHUFFLE);
        }
        break;

      case PREP_SPEAKER:
        switch (currentState) {
          case NONE:
          case PREP_SPEAKER:
          case PREP_VISION:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_SPEAKER);
        }
        break;

      case PREP_AMP:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_SHUFFLE:
          case PREP_AMP_SHOOTER:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_AMP);
        }
        break;

      case PREP_AMP_SHOOTER:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_AMP_SHOOTER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_SPIKE:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_AMP_SHOOTER);
        }
        break;

      case PREP_SPIKE:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPIKE:
          case PREP_SPEAKER:
          case PREP_VISION:
          case PREP_AMP_SHOOTER:
          case PREP_WING:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_SPIKE);
        }
        break;

      case PREP_WING:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_WING:
          case PREP_VISION:
          case PREP_SPIKE:
          case PREP_AMP_SHOOTER:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_WING);

        }
        break;

      case PREP_SUB_BACKWARDS:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_WING:
          case PREP_VISION:
          case PREP_SPIKE:
          case PREP_AMP_SHOOTER:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_SUB_BACKWARDS);

        }
        break;

      case PREP_VISION:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_WING:
          case PREP_VISION:
          case PREP_SPIKE:
          case PREP_AMP_SHOOTER:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepVision(subStateMachine, subDrivetrain, subShooter);

        }
        break;

      case PREP_NONE:
        switch (currentState) {
          case NONE:
          case STORE_FEEDER:
          case PREP_SHUFFLE:
          case PREP_SPEAKER:
          case PREP_WING:
          case PREP_VISION:
          case PREP_SPIKE:
          case PREP_AMP_SHOOTER:
          case PREP_AMP:
          case PREP_NONE:
          case PREP_SUB_BACKWARDS:
            return new PrepTargetState(subElevator, subStateMachine, subShooter, TargetState.PREP_NONE);
          case CLIMBING:
            if (subClimber.isClimberAtPosition(constClimber.BACKWARD_LIMIT)) {
              return new NoneState(subStateMachine, subClimber, subElevator, subIntake, subShooter, subTransfer);
            } else {
              return Commands
                  .print("Attempted to cancel CLIMBING, but the CLIMBER is UP! Please move the climber first :p");
            }

        }
    }
    return Commands.print("ITS SO OVER D: Invalid State Provided :3");
  }

  public Command tryTargetState(StateMachine subStateMachine, Intake subIntake,
      Shooter subShooter, Transfer subTransfer, Elevator subElevator, Drivetrain subDrivetrain) {
    if (currentTargetState.equals(TargetState.PREP_VISION)) {
      return new PrepVision(subStateMachine, subDrivetrain, subShooter);
    }
    return new PrepTargetState(subElevator, subStateMachine, subShooter, currentTargetState);
  }

  /**
   * Determines if our current robot state is also a target state.
   */
  public boolean isCurrentStateTargetState() {
    return isGivenStateTargetState(currentState);
  }

  public boolean isGivenStateTargetState(RobotState givenState) {
    for (TargetState targetState : TargetState.values()) {
      RobotState possibleRobotState = constStateMachine.TARGET_TO_ROBOT_STATE.get(targetState);
      if (givenState.equals(possibleRobotState)) {
        return true;
      }
    }

    return false;
  }

  public static enum RobotState {
    NONE,
    INTAKING,
    INTAKE_SOURCE,
    STORE_FEEDER,
    PREP_NONE,
    PREP_AMP,
    PREP_AMP_SHOOTER,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_VISION,
    PREP_SPIKE,
    PREP_WING,
    PREP_SUB_BACKWARDS,
    CLIMBING,
    SHOOTING,
    EJECTING
  }

  public static enum TargetState {
    PREP_NONE,
    PREP_AMP_SHOOTER,
    PREP_AMP,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_SUB_BACKWARDS,
    PREP_SPIKE,
    PREP_VISION,
    PREP_WING
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());

  }
}
