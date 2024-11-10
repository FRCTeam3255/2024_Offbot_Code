// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotMap.mapTransfer;

public class Transfer extends SubsystemBase {
  TalonFX feederMotor;
  TalonFXConfiguration feederConfig = new TalonFXConfiguration();
  DigitalInput noteSensor;
  boolean hasGamePiece;

  /** Creates a new Transfer. */
  public Transfer() {
    feederMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN, "rio");

    noteSensor = new DigitalInput(mapTransfer.NOTE_SENSOR_DIO);

    configure();
  }

  public void configure() {
    // -- Feeder Motor --
    feederConfig.MotorOutput.Inverted = constTransfer.MOTOR_INVERT;
    feederConfig.MotorOutput.NeutralMode = constTransfer.FEEDER_NEUTRAL_MODE;

    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = constTransfer.ENABLE_CURRENT_LIMITING;
    feederConfig.CurrentLimits.SupplyCurrentLimit = constTransfer.CURRENT_LIMIT;
    feederConfig.CurrentLimits.SupplyCurrentThreshold = constTransfer.CURRENT_THRESH;
    feederConfig.CurrentLimits.SupplyTimeThreshold = constTransfer.CURRENT_TIME_THRESH;

    feederMotor.getConfigurator().apply(feederConfig);
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void setFeederNeutralOutput() {
    feederMotor.setControl(new NeutralOut());
  }

  public boolean getGamePieceStored() {
    boolean noteSensorValue = (constTransfer.NOTE_SENSOR_INVERT) ? !noteSensor.get() : noteSensor.get();
    return (noteSensorValue || hasGamePiece);
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
  }
}
