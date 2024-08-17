// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;

public class PrepShuffle extends Command {
  StateMachine subStateMachine;
  Shooter subShooter;

  /** Creates a new PrepShuffle. */
  public PrepShuffle(StateMachine subStateMachine, Shooter subShooter) {
    this.subStateMachine = subStateMachine;
    this.subShooter = subShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subStateMachine.getRobotState() == RobotState.STORE_FEEDER) {
      subStateMachine.setRobotState(RobotState.PREP_SHUFFLE);
      subShooter.setDesiredVelocities(constShooter.LEFT_SHUFFLE_VELOCITY, constShooter.RIGHT_SHUFFLE_VELOCITY);
    }

    // TODO: NULL POINTER EXCEPTION MY BELOVED!
    // This will be an interpolating tree map in Constants one day
    subShooter.setShooterPosition(Units.Rotations.zero());
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
