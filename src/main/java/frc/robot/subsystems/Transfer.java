// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotMap.mapTransfer;

public class Transfer extends SubsystemBase {
  TalonFX feederMotor;
  TalonFXConfiguration feederConfig = new TalonFXConfiguration();
  LaserCan noteSensor;
  boolean hasGamePiece;

  /** Creates a new Transfer. */
  public Transfer() {
    feederMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN, "rio");

    noteSensor = new LaserCan(mapTransfer.NOTE_SENSOR_CAN);

    configure();
  }

  public void configure() {
    // -- Feeder Motor --
    feederConfig.MotorOutput.Inverted = constTransfer.MOTOR_INVERT;
    feederConfig.MotorOutput.NeutralMode = constTransfer.FEEDER_NEUTRAL_MODE;

    feederConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    feederConfig.CurrentLimits.StatorCurrentLimit = 80;

    feederConfig.Voltage.PeakForwardVoltage = 12.0;
    feederConfig.Voltage.PeakReverseVoltage = -12.0;

    feederMotor.getConfigurator().apply(feederConfig);

    try {
      noteSensor.setTimingBudget(constTransfer.TIMING_BUDGET);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCAN could not be initialized :<");

    }
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void setFeederNeutralOutput() {
    feederMotor.setControl(new NeutralOut());
  }

  public boolean getGamePieceStored() {
    Measurement measurement = noteSensor.getMeasurement();
    if (hasGamePiece) {
      return hasGamePiece;
    }

    if (measurement != null) {
      return measurement.distance_mm <= constTransfer.PIECE_DETECTED_DIST_THRESH.in(Units.Millimeters);
    }

    return false;
  }

  public void setGamePieceCollected(boolean isCollected) {
    hasGamePiece = isCollected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Game Piece Stored", getGamePieceStored());
    SmartDashboard.putNumber("Transfer/Supply Current", feederMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Transfer/Stator Current", feederMotor.getStatorCurrent().getValueAsDouble());
    if (noteSensor.getMeasurement() != null) {
      SmartDashboard.putNumber("Transfer/Laser Can/Distance", noteSensor.getMeasurement().distance_mm);
      SmartDashboard.putNumber("Transfer/Laser Can/Ambient Light", noteSensor.getMeasurement().ambient);
      SmartDashboard.putNumber("Transfer/Laser Can/Timing Budget", noteSensor.getMeasurement().budget_ms);
    }
  }
}
