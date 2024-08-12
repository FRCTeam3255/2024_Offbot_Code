// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;

public class NoneState extends Command {
  StateMachine subStateMachine;
  Elevator subElevator;
  Intake subIntake;
  Shooter subShooter;
  Transfer subTransfer;

  /** Creates a new NoneState. */
  public NoneState(StateMachine subStateMachine, Elevator subElevator, Intake subIntake,
      Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.NONE);
    subElevator.setElevatorPosition(constElevator.BACKWARD_LIMIT);
    subIntake.setIntakeRollerSpeed(Units.Percent.of(0));
    subTransfer.setFeederSpeed(Units.Percent.of(0));
    subShooter.setDesiredVelocities(Units.RotationsPerSecond.of(0), Units.RotationsPerSecond.of(0));
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
