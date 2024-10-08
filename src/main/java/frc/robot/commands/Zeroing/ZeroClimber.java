// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constClimber;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends Command {
  Climber subClimber;

  Measure<Time> zeroingTimestamp;

  public ZeroClimber(Climber subClimber) {
    this.subClimber = subClimber;

    addRequirements(subClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subClimber.setSoftwareLimits(false, true);

    subClimber.setVoltage(Units.Volts.zero());
    zeroingTimestamp = Units.Seconds.zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subClimber.setVoltage(constClimber.ZEROING_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subClimber.setSoftwareLimits(true, true);

    // Stop all movement
    subClimber.setVoltage(Units.Volts.zero());

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      subClimber.setClimberSensorPosition(constClimber.ZEROED_POS);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If the current velocity is low enough to be considered as zeroed
    if (subClimber.getVelocity().lte(constClimber.ZEROED_VELOCITY)) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp.equals(Units.Seconds.zero())) {
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constClimber.ZEROED_TIME));
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    zeroingTimestamp = Units.Seconds.zero();
    return false;
  }
}
