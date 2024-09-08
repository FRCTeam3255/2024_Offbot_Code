// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;
import frc.robot.RobotPreferences.prefElevator;

public class Elevator extends SubsystemBase {
  TalonFX elevatorMotor, drainpipeMotor;
  TalonFXConfiguration elevatorConfig, drainpipeConfig;

  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new TalonFX(mapElevator.ELEVATOR_MOTOR_CAN, "rio");
    drainpipeMotor = new TalonFX(mapElevator.DRAINPIPE_MOTOR_CAN, "rio");

    elevatorConfig = new TalonFXConfiguration();
    drainpipeConfig = new TalonFXConfiguration();
    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

    configure();
  }

  public void configure() {
    // -- Elevator Motor --
    elevatorConfig.Feedback.SensorToMechanismRatio = constElevator.MOTOR_ROTATION_TO_METERS;
    elevatorConfig.MotorOutput.Inverted = constElevator.MOTOR_INVERT;
    elevatorConfig.Slot0.kP = prefElevator.elevatorShooterP.getValue();
    elevatorConfig.Slot0.kI = prefElevator.elevatorShooterI.getValue();
    elevatorConfig.Slot0.kD = prefElevator.elevatorShooterD.getValue();

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constElevator.FORWARD_LIMIT.in(Units.Rotations);

    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constElevator.FORWARD_LIMIT.in(Units.Rotations);

    elevatorMotor.getConfigurator().apply(elevatorConfig);

    // -- Drainpipe Motor --
    drainpipeMotor.getConfigurator().apply(drainpipeConfig);
  }

  public void setSoftwareLimits(boolean reverse, boolean forward) {
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    elevatorMotor.getConfigurator().apply(elevatorConfig);
  }

  public void setDrainpipeSpeed(Measure<Dimensionless> speed) {
    drainpipeMotor.set(speed.in(Units.Percent));
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setVoltage(Measure<Voltage> voltage) {
    elevatorMotor.setControl(voltageRequest.withOutput(voltage.in(Units.Volts)));
  }

  public void setElevatorPosition(Measure<Angle> position) {
    elevatorMotor.setControl(positionRequest.withPosition(position.in(Units.Rotations)));
  }

  /**
   * Sets the current position of the elevator motor to read as the given value
   */
  public void setElevatorSensorPosition(Measure<Distance> position) {
    elevatorMotor.setPosition(position.in(Units.Meters));
  }

  /**
   * @return The current position of the elevator in rotations
   */
  public Measure<Angle> getElevatorPosition() {
    return Units.Rotations.of(elevatorMotor.getPosition().getValueAsDouble());
  }

  public Measure<Velocity<Distance>> getVelocity() {
    return Units.MetersPerSecond.of(elevatorMotor.getVelocity().getValueAsDouble());
  }

  /**
   * @return If the elevator position is within tolerance of desired position
   */
  public boolean isElevatorAtPosition(Measure<Angle> position) {
    return (Math.abs(getElevatorPosition().minus(position).in(Units.Rotations)) < constElevator.AT_POSITION_TOLERANCE
        .in(Units.Rotations));

    // TODO: test if the code below works and makes our lives easier
    // return
    // (getElevatorPosition().minus(position)).lte(constElevator.AT_POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
