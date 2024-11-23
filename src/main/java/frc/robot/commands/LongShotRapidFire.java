// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Transfer;

public class LongShotRapidFire extends Command {
  /**
   * THIS IS FOR AMERICA'S GOT TALENT ACT PURPOSES.
   * Breaks the rules :3
   */
  StateMachine subStateMachine;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  public LongShotRapidFire(StateMachine subStateMachine, Intake subIntake, Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subIntake = subIntake;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subIntake.setIntakeRollerSpeed(constIntake.INTAKING_SPEED);
    subTransfer.setFeederSpeed(0.1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subShooter.readyToShoot()) {
      subTransfer.setFeederSpeed(constTransfer.SHOOTING_SPEED);
    }
    subShooter.setDesiredPosition(constShooter.LONG_SHOT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subIntake.setIntakeRollerSpeed(Units.Percent.zero());
    subTransfer.setFeederSpeed(0);
    subShooter.setShootingNeutralOutput();
    subShooter.setPivotNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
