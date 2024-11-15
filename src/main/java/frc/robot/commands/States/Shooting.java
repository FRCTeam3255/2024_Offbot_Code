// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constTransfer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

public class Shooting extends Command {
  StateMachine subStateMachine;
  Elevator subElevator;
  Shooter subShooter;
  Transfer subTransfer;
  Intake subIntake;

  /** Creates a new Shooting. */
  public Shooting(StateMachine subStateMachine, Elevator subElevator, Shooter subShooter, Transfer subTransfer,
      Intake subIntake) {
    this.subStateMachine = subStateMachine;
    this.subElevator = subElevator;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subIntake = subIntake;

    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subStateMachine.getRobotState() == RobotState.PREP_AMP
        || subStateMachine.getRobotState() == RobotState.CLIMBING) {
      // Score through drainpipe if in PREP_AMP or CLIMBING (for trap)
      subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_SCORE_AMP_SPEED);
      subTransfer.setFeederSpeed(constTransfer.SHOOTING_SPEED);
      subStateMachine.setRobotState(RobotState.SHOOTING);
      subTransfer.setGamePieceCollected(false);
    } else {
      // Otherwise, shoot through the flywheels if they are up to speed
      if (subShooter.readyToShoot()) {
        subTransfer.setFeederSpeed(constTransfer.SHOOTING_SPEED);
        subStateMachine.setRobotState(RobotState.SHOOTING);
        subIntake.setIntakeRollerSpeed(constIntake.INTAKING_SPEED);
        subTransfer.setGamePieceCollected(false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If we don't have a game piece anymore, set the target state back to NONE
    if (!subTransfer.getGamePieceStored()) {
      subStateMachine.setQueueState(TargetState.PREP_NONE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
