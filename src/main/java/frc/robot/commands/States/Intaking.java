// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Intaking extends Command {
  StateMachine subStateMachine;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  /** Creates a new Intake. */
  public Intaking(StateMachine subStateMachine, Intake subIntake, Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subIntake = subIntake;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.INTAKING);
    subIntake.setIntakeRollerSpeed(constIntake.INTAKING_SPEED);
    subTransfer.setFeederSpeed(constTransfer.INTAKING_SPEED);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subShooter.isShooterAtPosition(constShooter.PIVOT_BACKWARD_LIMIT)) {
      subShooter.setPivotPosition(constShooter.PIVOT_BACKWARD_INTAKE_LIMIT);
    } else if (subShooter.isShooterAtPosition(constShooter.PIVOT_FORWARD_LIMIT)) {
      subShooter.setPivotPosition(constShooter.PIVOT_FORWARD_INTAKE_LIMIT);
    } else {
      subShooter.setPivotPosition(subShooter.getShooterPosition());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.getGamePieceCollected();
  }
}
