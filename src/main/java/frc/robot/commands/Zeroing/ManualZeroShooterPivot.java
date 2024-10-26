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
  Measure<Velocity<Angle>> lastRotorVelocity = Units.RotationsPerSecond.of(0);

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

    // Check if we have raised the shooter above a certain speed
    if (subShooter.getPivotRotorVelocity().gte(constShooter.MANUAL_ZEROING_START_VELOCITY) || attemptingZeroing) {
      // Enter zeroing mode!
      if (!attemptingZeroing) {
        attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());

        System.out.println("Zeroing Start!");
      }

      // Check if time elapsed since previous zeroing is too high - if true, then exit
      // zeroing mode :(
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constShooter.ZEROING_TIMEOUT)) {
        attemptingZeroing = false;
        System.out.println("Zeroing Failed :(");

      } else {
        boolean current = subShooter.getPivotCurrent().minus(lastCurrent)
            .gte(constShooter.MANUAL_ZEROING_DELTA_CURRENT);
        boolean rotorVelocity = subShooter.getPivotRotorVelocity().minus(lastRotorVelocity)
            .gte(constShooter.MANUAL_ZEROING_DETLA_VELOCITY);

        if (current && rotorVelocity) {
          zeroingSuccess = true;
        } else {
          lastCurrent = subShooter.getPivotCurrent();
          lastRotorVelocity = subShooter.getPivotRotorVelocity();
        }
      }

    }

  }

  @Override
  public void end(boolean interrupted) {
    Shooter.hasZeroed = true;
    subShooter.setPivotSensorAngle(constShooter.ZEROED_ANGLE);
    System.out.println("Zeroing Successful!!!! Yippee and hooray!!! :3");
  }

  @Override
  public boolean isFinished() {
    return zeroingSuccess;
  }
}
