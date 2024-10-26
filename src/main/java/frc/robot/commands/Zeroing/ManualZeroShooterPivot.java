// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Shooter;

public class ManualZeroShooterPivot extends Command {
  Shooter subShooter;

  boolean attemptingZeroing = false;
  boolean zeroingSuccess = false;
  Measure<Time> zeroingTimestamp = Units.Seconds.zero();

  Measure<Voltage> lastCurrent = Units.Volts.of(0);

  public ManualZeroShooterPivot(Shooter subShooter) {
    this.subShooter = subShooter;

    addRequirements(subShooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("this is running <3",
        subShooter.getPivotCurrent().baseUnitMagnitude());

    SmartDashboard.putBoolean("boolean",
        subShooter.getPivotCurrent().lte(constShooter.MANUAL_ZEROING_START_CURRENT));

    // Check if we have raised the shooter above a certain angle
    if (subShooter.getPivotCurrent().lte(constShooter.MANUAL_ZEROING_START_CURRENT) || attemptingZeroing) {
      // Enter zeroing mode!
      if (!attemptingZeroing) {
        attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());

        Commands.deferredProxy(() -> Commands.print("Zeroing Start!"));
      }

      // Check if time elapsed since previous zeroing is too high - if true, then exit
      // zeroing mode :(
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constShooter.ZEROED_TIME)) {
        attemptingZeroing = false;
        Commands.deferredProxy(() -> Commands.print("Zeroing Failed :("));

      } else {
        // Otherwise, we may be zeroed! Check some mystery numbers
        SmartDashboard.putNumber("lte thing", subShooter.getPivotCurrent().minus(lastCurrent).baseUnitMagnitude());
        if (subShooter.getPivotCurrent().minus(lastCurrent).lte(constShooter.MANUAL_ZEROING_DELTA_CURRENT)) {
          zeroingSuccess = true;
        } else {
          lastCurrent = subShooter.getPivotCurrent();
        }
      }

    }

  }

  @Override
  public void end(boolean interrupted) {
    Shooter.hasZeroed = true;
    subShooter.setPivotSensorAngle(constShooter.ZEROED_ANGLE);
    Commands.deferredProxy(() -> Commands.print("Zeroing Successful!!!!"));
  }

  @Override
  public boolean isFinished() {
    return zeroingSuccess;
  }
}
