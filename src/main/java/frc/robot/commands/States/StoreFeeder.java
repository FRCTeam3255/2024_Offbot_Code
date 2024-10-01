// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;

public class StoreFeeder extends Command {
  StateMachine subStateMachine;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  /** Creates a new StoreTransfer. */
  public StoreFeeder(StateMachine subStateMachine, Intake subIntake, Transfer subTransfer, Shooter subShooter) {
    this.subStateMachine = subStateMachine;
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.STORE_FEEDER);
    subIntake.setIntakeRollerSpeed(Units.Percent.zero());
    subTransfer.setFeederSpeed(0);
    subShooter.setShooterPercentOutput(Units.Percent.zero());

    // This allows StoreFeeder to effectively also act as our "Unalive Shooter"
    // command from last year
    // We don't care if it actually gets there- just that its going there
    subShooter.setPivotPosition(constShooter.PIVOT_BACKWARD_LIMIT);
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
