// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;

public class IntakeSource extends Command {
  StateMachine subStateMachine;
  Shooter subShooter;
  Transfer subTransfer;

  boolean hasGamePiece;

  /** Creates a new IntakeSource. */
  public IntakeSource(StateMachine subStateMachine, Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    hasGamePiece = false;

    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.INTAKE_SOURCE);
    subTransfer.setFeederSpeed(constTransfer.INTAKE_SOURCE_SPEED);
    subShooter.setShooterPercentOutput(constShooter.INTAKE_SOURCE_SPEED);
    subShooter.setPivotPosition(constShooter.PIVOT_BACKWARD_INTAKE_LIMIT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subTransfer.getGamePieceCollected()) {
      hasGamePiece = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTransfer.setFeederSpeed(0);
    subShooter.setShootingNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Since the sensor senses the back of the note, command will end when we know
    // we have a game piece but when the sensor doesn't sense the note anymore (it's
    // far back enough to not touch the flywheels)
    return (hasGamePiece && !subTransfer.getGamePieceCollected());
  }

  public boolean getIntakeSourceGamePiece() {
    return hasGamePiece;
  }
}
