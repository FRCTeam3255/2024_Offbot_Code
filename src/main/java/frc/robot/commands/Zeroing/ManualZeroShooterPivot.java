// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Shooter;

public class ManualZeroShooterPivot extends Command {
  Shooter subShooter;

  boolean attemptingZeroing = false;
  boolean zeroingSuccess = false;
  Measure<Time> zeroingTimestamp = Units.Seconds.zero();

  public ManualZeroShooterPivot(Shooter subShooter) {
    this.subShooter = subShooter;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Check if we have raised the shooter above a certain angle
    if (subShooter.getShooterPosition().gte(constShooter.MANUAL_ZEROING_START_ANGLE) || attemptingZeroing) {
      // Enter zeroing mode!
      if (!attemptingZeroing) {
        attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        Commands.deferredProxy(() -> subShooter.playZeroingStart());
      }

      // Check if time elapsed since previous zeroing is too high - if true, then exit
      // zeroing mode :(
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constShooter.ZEROED_TIME)) {
        attemptingZeroing = false;
        Commands.deferredProxy(() -> subShooter.playZeroingFailed());

      } else {
        // Otherwise, we may be zeroed! Check some mystery numbers
        // TODO: Add the mystery
        if (true) {
          zeroingSuccess = true;
        }
      }

    }

  }

  @Override
  public void end(boolean interrupted) {
    Shooter.hasZeroed = true;
    Commands.deferredProxy(() -> subShooter.playZeroingSuccess());
  }

  @Override
  public boolean isFinished() {
    return zeroingSuccess;
  }
}
