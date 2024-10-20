// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  DigitalInput noteSensor;

  TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new TalonFX(mapIntake.ROLLER_CAN, "rio");
    noteSensor = new DigitalInput(mapIntake.NOTE_SENSOR_DIO);

    configure();
  }

  public void configure() {
    rollerConfig.MotorOutput.Inverted = constIntake.MOTOR_INVERT;

    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = constIntake.ENABLE_CURRENT_LIMITING;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = constIntake.CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = constIntake.CURRENT_THRESH;
    rollerConfig.CurrentLimits.SupplyTimeThreshold = constIntake.CURRENT_TIME_THRESH;

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  public void setIntakeRollerSpeed(Measure<Dimensionless> speed) {
    rollerMotor.set(speed.in(Units.Percent));
  }

  /**
   * Sets the rollers to neutral.
   */
  public void setRollerNeutralOutput() {
    rollerMotor.setControl(new NeutralOut());
  }

  public boolean getGamePieceCollected() {
    return (constIntake.NOTE_SENSOR_INVERT) ? !noteSensor.get() : noteSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Game Piece Detected", getGamePieceCollected());
  }

}
