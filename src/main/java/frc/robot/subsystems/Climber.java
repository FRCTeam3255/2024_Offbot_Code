// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;
  TalonFXConfiguration climberConfig = new TalonFXConfiguration();
  VoltageOut voltageRequest;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");
    voltageRequest = new VoltageOut(0);

    configure();
  }

  public void configure() {
    climberConfig.Feedback.SensorToMechanismRatio = constClimber.MOTOR_ROTATION_TO_METERS;
    climberConfig.MotorOutput.Inverted = constClimber.MOTOR_INVERT;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constClimber.FORWARD_LIMIT.in(Units.Rotations);

    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constClimber.FORWARD_LIMIT.in(Units.Rotations);
    climberMotor.getConfigurator().apply(climberConfig);
  }

  public void setSoftwareLimits(boolean reverse, boolean forward) {
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    climberMotor.getConfigurator().apply(climberConfig);
  }

  public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  public void setVoltage(Measure<Voltage> voltage) {
    climberMotor.setControl(voltageRequest.withOutput(voltage.in(Units.Volts)));
  }

  /**
   * Sets the current position of the climber motor to read as the given value
   */
  public void setClimberSensorPosition(Measure<Distance> position) {
    climberMotor.setPosition(position.in(Units.Meters));
  }

  public Measure<Velocity<Distance>> getVelocity() {
    return Units.MetersPerSecond.of(climberMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
