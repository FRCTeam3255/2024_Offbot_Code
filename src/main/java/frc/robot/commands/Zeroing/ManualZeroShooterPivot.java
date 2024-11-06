// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Shooter;

public class ManualZeroShooterPivot extends Command {
  Shooter subShooter;

  boolean zeroingSuccess = false;
  Measure<Time> zeroingTimestamp = Units.Seconds.zero();

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
    // Check if we have raised the shooter above a certain speed
    if (subShooter.getPivotRotorVelocity().gte(constShooter.MANUAL_ZEROING_START_VELOCITY)
        || Shooter.attemptingZeroing) {
      // Enter zeroing mode!
      if (!Shooter.attemptingZeroing) {
        Shooter.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Shooter Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constShooter.ZEROING_TIMEOUT)) {
        Shooter.attemptingZeroing = false;
        System.out.println("Shooter Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = subShooter.getPivotRotorVelocity().minus(lastRotorVelocity)
            .lte(constShooter.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = subShooter.getPivotRotorVelocity();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      Shooter.hasZeroed = true;
      subShooter.setPivotSensorAngle(constShooter.ZEROED_ANGLE);
      System.out.println("Shooter Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Shooter was never zeroed :((( blame eli");
    }
  }

  @Override
  public boolean isFinished() {
    boolean rotorVelocityIsZero = subShooter.getPivotRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    SmartDashboard.putBoolean("Zeroing/Pivot/Is Rotor Velocity Zero", rotorVelocityIsZero);
    return zeroingSuccess && rotorVelocityIsZero;
  }
}
