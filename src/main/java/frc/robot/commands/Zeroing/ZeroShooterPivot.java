// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Shooter;

public class ZeroShooterPivot extends Command {
  Shooter subShooter;

  Measure<Time> zeroingTimestamp;
  boolean hasZeroed = false;

  public ZeroShooterPivot(Shooter subShooter) {
    this.subShooter = subShooter;
    addRequirements(subShooter);
  }

  @Override
  public void initialize() {
    subShooter.setPivotSoftwareLimits(false, true);

    subShooter.setPivotVoltage(Units.Volts.zero());
    zeroingTimestamp = Units.Seconds.zero();
    hasZeroed = Shooter.hasZeroed;
  }

  @Override
  public void execute() {
    subShooter.setPivotVoltage(constShooter.ZEROING_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setPivotSoftwareLimits(true, true);

    // Stop all movement
    subShooter.setPivotVoltage(Units.Volts.zero());

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      subShooter.setPivotSensorAngle(constShooter.ZEROED_ANGLE);
      Shooter.hasZeroed = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasZeroed) {
      return true;
    }

    // If the current velocity is low enough to be considered as zeroed
    if (subShooter.getPivotVelocity().gte(constShooter.ZEROED_VELOCITY)) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp.equals(Units.Seconds.zero())) {
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constShooter.ZEROED_TIME));
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    zeroingTimestamp = Units.Seconds.zero();
    return false;
  }
}
