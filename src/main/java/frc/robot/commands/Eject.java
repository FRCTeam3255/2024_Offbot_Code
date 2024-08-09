// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotContainer.RobotState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class Eject extends Command {
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  boolean canEject = true;

  /** Creates a new Eject. */
  public Eject(Intake subIntake, Transfer subTransfer, Shooter subShooter) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subIntake, subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (RobotContainer.getRobotState()) {
      case STORE_FEEDER:
        subIntake.setIntakeRollerSpeed(constIntake.EJECTING_SPEED);
        subTransfer.setFeederSpeed(constTransfer.EJECTING_SPEED);
        break;
      case PREP_SHUFFLE:
        subIntake.setIntakeRollerSpeed(constIntake.EJECTING_SPEED);
        subTransfer.setFeederSpeed(constTransfer.EJECTING_SPEED);
        break;
      case PREP_SPEAKER:
        subIntake.setIntakeRollerSpeed(constIntake.EJECTING_SPEED);
        subTransfer.setFeederSpeed(constTransfer.EJECTING_SPEED);
        break;
      default: // if robot is in a state where it can't eject (or we don't want it to), then do
               // nothing
        subIntake.setIntakeRollerSpeed(Units.Percent.of(0));
        subTransfer.setFeederSpeed(Units.Percent.of(0));
        canEject = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (canEject) {
      RobotContainer.setRobotState(RobotState.EJECTING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (canEject) {
      RobotContainer.setRobotState(RobotState.NONE);

      subIntake.setRollerNeutralOutput();
      subTransfer.setFeederNeutralOutput();
      subShooter.setShootingNeutralOutput();
      subShooter.setShooterPosition(constShooter.PIVOT_BACKWARD_LIMIT);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
