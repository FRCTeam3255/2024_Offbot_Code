// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;
import frc.robot.RobotPreferences.prefElevator;

public class Elevator extends SubsystemBase {
  TalonFX elevatorMotor, drainpipeMotor;
  TalonFXConfiguration elevatorConfig, drainpipeConfig;
  DigitalInput noteSensor;

  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  public static boolean attemptingZeroing = false;
  public static boolean hasZeroed = false;

  public Elevator() {
    elevatorMotor = new TalonFX(mapElevator.ELEVATOR_MOTOR_CAN, "rio");
    drainpipeMotor = new TalonFX(mapElevator.DRAINPIPE_MOTOR_CAN, "rio");
    noteSensor = new DigitalInput(mapElevator.NOTE_SENSOR_DIO);

    elevatorConfig = new TalonFXConfiguration();
    drainpipeConfig = new TalonFXConfiguration();
    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

    configure();
  }

  public void configure() {
    // -- Elevator Motor --
    elevatorConfig.Feedback.SensorToMechanismRatio = constElevator.MOTOR_ROTATION_TO_METERS;
    elevatorConfig.MotorOutput.NeutralMode = constElevator.ELEVATOR_NEUTRAL_MODE;
    elevatorConfig.MotorOutput.Inverted = constElevator.MOTOR_INVERT;
    elevatorConfig.Slot0.GravityType = constElevator.ELEVATOR_GRAVITY_TYPE;
    elevatorConfig.Slot0.StaticFeedforwardSign = constElevator.ELEVATOR_STATIC_FFWD_SIGN;
    elevatorConfig.Slot0.kG = prefElevator.elevatorG;
    elevatorConfig.Slot0.kS = prefElevator.elevatorS;
    elevatorConfig.Slot0.kP = prefElevator.elevatorP;
    elevatorConfig.Slot0.kI = prefElevator.elevatorI;
    elevatorConfig.Slot0.kD = prefElevator.elevatorD;

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constElevator.FORWARD_LIMIT.in(Units.Meters);

    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constElevator.BACKWARD_LIMIT.in(Units.Meters);

    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    elevatorConfig.CurrentLimits.SupplyTimeThreshold = 0.01;

    elevatorMotor.getConfigurator().apply(elevatorConfig);

    // -- Drainpipe Motor --
    drainpipeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    drainpipeConfig.CurrentLimits.StatorCurrentLimit = 80;

    drainpipeConfig.Voltage.PeakForwardVoltage = 12.0;
    drainpipeConfig.Voltage.PeakReverseVoltage = -12.0;

    drainpipeMotor.getConfigurator().apply(drainpipeConfig);
  }

  public void setSoftwareLimits(boolean reverse, boolean forward) {
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    elevatorMotor.getConfigurator().apply(elevatorConfig);
  }

  public void setDrainpipeSpeed(double speed) {
    drainpipeMotor.set(speed);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setVoltage(Measure<Voltage> voltage) {
    elevatorMotor.setControl(voltageRequest.withOutput(voltage.in(Units.Volts)));
  }

  public void setElevatorPosition(Measure<Distance> position) {
    elevatorMotor.setControl(positionRequest.withPosition(position.in(Units.Meters)));
  }

  /**
   * Sets the current position of the elevator motor to read as the given value
   */
  public void setElevatorSensorPosition(Measure<Distance> position) {
    elevatorMotor.setPosition(position.in(Units.Meters));
  }

  /**
   * @return The current position of the elevator in meters
   */
  public Measure<Distance> getElevatorPosition() {
    return Units.Meters.of(elevatorMotor.getPosition().getValueAsDouble());
  }

  public Measure<Velocity<Distance>> getVelocity() {
    return Units.MetersPerSecond.of(elevatorMotor.getVelocity().getValueAsDouble());
  }

  public Measure<Velocity<Angle>> getRotorVelocity() {
    return Units.RotationsPerSecond.of(elevatorMotor.getRotorVelocity().getValueAsDouble());
  }

  public Measure<Voltage> getCurrent() {
    return Units.Volts.of(elevatorMotor.getStatorCurrent().getValueAsDouble());
  }

  /**
   * @return If the elevator position is within tolerance of desired position
   */
  public boolean isElevatorAtPosition(Measure<Distance> position) {
    return (Math.abs(getElevatorPosition().minus(position).in(Units.Meters)) < constElevator.AT_POSITION_TOLERANCE
        .in(Units.Meters));

    // TODO: test if the code below works and makes our lives easier
    // return
    // (getElevatorPosition().minus(position)).lte(constElevator.AT_POSITION_TOLERANCE);
  }

  public boolean isSafeToMoveShooterAboveLimit() {
    return getElevatorPosition().gte(constElevator.SHOOTER_ABLE_TO_MOVE_LIMIT);
  }

  public boolean getGamePieceStored() {
    return (constElevator.NOTE_SENSOR_INVERT) ? !noteSensor.get() : noteSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator/Position", getElevatorPosition().in(Units.Meters));
    SmartDashboard.putBoolean("Elevator/Safe To Move Shooter", isSafeToMoveShooterAboveLimit());
    SmartDashboard.putNumber("Elevator/Stator Current", elevatorMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Rotor Velocity", elevatorMotor.getRotorVelocity().getValueAsDouble());

    SmartDashboard.putBoolean("Zeroing/Elevator/Attempting Zeroing", attemptingZeroing);
    SmartDashboard.putBoolean("Zeroing/Elevator/Has Zeroed", hasZeroed);

  }
}
