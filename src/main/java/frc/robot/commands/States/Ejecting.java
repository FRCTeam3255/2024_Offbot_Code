// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;

public class Ejecting extends Command {
  StateMachine subStateMachine;
  Intake subIntake;
  Elevator subElevator;
  Transfer subTransfer;
  Shooter subShooter;

  /** Creates a new Eject. */
  public Ejecting(StateMachine subStateMachine, Intake subIntake, Elevator subElevator, Transfer subTransfer,
      Shooter subShooter) {
    this.subStateMachine = subStateMachine;
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subElevator = subElevator;
    this.subShooter = subShooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.EJECTING);
    subIntake.setIntakeRollerSpeed(constIntake.EJECTING_SPEED);
    subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_EJECTING_SPEED);
    subTransfer.setFeederSpeed(constTransfer.EJECTING_SPEED);
    subShooter.setDesiredVelocities(constShooter.EJECTING_SPEED, constShooter.EJECTING_SPEED);
    subShooter.getUpToSpeed();
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
    return false;
  }
}
