// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.TargetState;

public class PrepTargetState extends InstantCommand {
  StateMachine subStateMachine;
  Shooter subShooter;
  TargetState desiredTargetState;

  Measure<Angle> desiredPivotAngle;
  Measure<Velocity<Angle>> desiredLeftVelocity, desiredRightVelocity;

  /** Creates a new PrepTargetState. */
  public PrepTargetState(StateMachine subStateMachine, Shooter subShooter, TargetState desiredTargetState) {
    this.subStateMachine = subStateMachine;
    this.subShooter = subShooter;
    this.desiredTargetState = desiredTargetState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: Determine the desired Robot state to go to based on the target state
    // Determine if we should actually go to that state- Im pretty sure this will
    // always be true but i need to read PrepSpeaker again haha
    // TODO: Add Dictionary for TargetState -> Pivot angle, velocities

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
