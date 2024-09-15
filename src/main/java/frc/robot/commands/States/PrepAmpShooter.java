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

public class PrepAmpShooter extends Command {
  StateMachine subStateMachine;
  Shooter subShooter;

  /** Creates a new PrepSpeaker. */
  public PrepAmpShooter(StateMachine subStateMachine, Shooter subShooter) {
    this.subStateMachine = subStateMachine;
    this.subShooter = subShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Only go to the real PREP_SPEAKER state if the robot is in these states:
    switch (subStateMachine.getRobotState()) {
      case STORE_FEEDER:
      case PREP_AMP:
        subStateMachine.setRobotState(RobotState.PREP_AMP);
    }

    // Otherwise, just set the angle of the shooter
    subShooter.setPivotPosition(constShooter.PIVOT_AMP_ANGLE);
    subShooter.setDesiredVelocities(constShooter.LEFT_AMP_VELOCITY,
        constShooter.RIGHT_AMP_VELOCITY);

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
    return true;
  }
}
