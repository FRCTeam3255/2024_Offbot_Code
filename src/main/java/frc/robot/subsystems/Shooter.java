// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor, rightMotor, pivotMotor;
  TalonFXConfiguration leftConfig, rightConfig, pivotConfig;

  MotionMagicVelocityVoltage motionMagicRequest;
  PositionVoltage positionRequest;

  VelocityVoltage velocityRequest;
  VoltageOut voltageRequest;
  boolean leftInvert, rightInvert, pivotInvert;

  private boolean ignoreFlywheelSpeed = false;
  private Measure<Velocity<Angle>> desiredLeftVelocity = Units.RotationsPerSecond.of(0);
  private Measure<Velocity<Angle>> desiredRightVelocity = Units.RotationsPerSecond.of(0);

  public Shooter() {
    leftMotor = new TalonFX(mapShooter.SHOOTER_LEFT_MOTOR_CAN, "rio");
    rightMotor = new TalonFX(mapShooter.SHOOTER_RIGHT_MOTOR_CAN, "rio");
    pivotMotor = new TalonFX(mapShooter.SHOOTER_PIVOT_MOTOR_CAN, "rio");

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();
    pivotConfig = new TalonFXConfiguration();

    leftInvert = constShooter.LEFT_INVERT;
    rightInvert = constShooter.RIGHT_INVERT;
    pivotInvert = constShooter.PIVOT_INVERT;

    voltageRequest = new VoltageOut(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    motionMagicRequest = new MotionMagicVelocityVoltage(0);
    positionRequest = new PositionVoltage(0).withSlot(0);

    configure();
  }

  public void configure() {
    leftConfig.Slot0.kV = prefShooter.leftShooterV.getValue();
    leftConfig.Slot0.kS = prefShooter.leftShooterS.getValue();
    leftConfig.Slot0.kA = prefShooter.leftShooterA.getValue();
    leftConfig.Slot0.kP = prefShooter.leftShooterP.getValue();
    leftConfig.Slot0.kI = prefShooter.leftShooterI.getValue();
    leftConfig.Slot0.kD = prefShooter.leftShooterD.getValue();

    leftConfig.MotionMagic.MotionMagicAcceleration = 400;
    leftConfig.MotionMagic.MotionMagicJerk = 4000;

    rightConfig.Slot0.kV = prefShooter.rightShooterV.getValue();
    rightConfig.Slot0.kS = prefShooter.rightShooterS.getValue();
    rightConfig.Slot0.kA = prefShooter.rightShooterA.getValue();
    rightConfig.Slot0.kP = prefShooter.rightShooterP.getValue();
    rightConfig.Slot0.kI = prefShooter.rightShooterI.getValue();
    rightConfig.Slot0.kD = prefShooter.rightShooterD.getValue();

    rightConfig.MotionMagic.MotionMagicAcceleration = 400;
    rightConfig.MotionMagic.MotionMagicJerk = 4000;

    pivotConfig.Feedback.SensorToMechanismRatio = constShooter.PIVOT_GEAR_RATIO;
    pivotConfig.Slot0.kP = prefShooter.leftShooterP.getValue();
    pivotConfig.Slot0.kI = prefShooter.leftShooterI.getValue();
    pivotConfig.Slot0.kD = prefShooter.leftShooterD.getValue();

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constShooter.PIVOT_FORWARD_LIMIT.in(Units.Rotations);

    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constShooter.PIVOT_FORWARD_LIMIT.in(Units.Rotations);

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    pivotMotor.getConfigurator().apply(pivotConfig);

    leftMotor.setInverted(leftInvert);
    rightMotor.setInverted(rightInvert);
    pivotMotor.setInverted(pivotInvert);
  }

  public void setPivotSoftwareLimits(boolean reverse, boolean forward) {
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setInverted(pivotInvert);
  }

  /**
   * Sets both shooting motors to try to get to their previously-assigned desired
   * speeds.
   */
  public void getUpToSpeed() {
    if (desiredLeftVelocity.in(Units.RotationsPerSecond) == 0
        && desiredRightVelocity.in(Units.RotationsPerSecond) == 0) {
      setShootingNeutralOutput();
    } else {
      leftMotor.setControl(motionMagicRequest.withVelocity(desiredLeftVelocity.in(Units.RotationsPerSecond)));
      rightMotor.setControl(motionMagicRequest.withVelocity(desiredRightVelocity.in(Units.RotationsPerSecond)));
    }
  }

  /**
   * Sets all of the shooting motors to neutral.
   */
  public void setShootingNeutralOutput() {
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  /**
   * @return The current velocity of the left shooter motor.
   */
  public Measure<Velocity<Angle>> getLeftShooterVelocity() {
    return Units.RotationsPerSecond.of(leftMotor.getVelocity().getValueAsDouble());
  }

  /**
   * @return The current velocity of the right shooter motor.
   */
  public Measure<Velocity<Angle>> getRightShooterVelocity() {
    return Units.RotationsPerSecond.of(rightMotor.getVelocity().getValueAsDouble());
  }

  public Measure<Velocity<Angle>> getPivotVelocity() {
    return Units.RotationsPerSecond.of(pivotMotor.getVelocity().getValueAsDouble());
  }

  /**
   * @return If the left shooter motor is at its desired velocity
   */
  public boolean isLeftShooterUpToSpeed() {
    return (getLeftShooterVelocity().minus(desiredLeftVelocity)).lte(constShooter.UP_TO_SPEED_TOLERANCE);
  }

  /**
   * @return If the right shooter motor is at its desired velocity
   */
  public boolean isRightShooterUpToSpeed() {
    return (getRightShooterVelocity().minus(desiredRightVelocity)).lte(constShooter.UP_TO_SPEED_TOLERANCE);
  }

  /**
   * @return If both motors have a non-zero desired velocity and are at their
   *         desired velocities
   */
  public boolean areBothShootersUpToSpeed() {
    return (isLeftShooterUpToSpeed()
        && isRightShooterUpToSpeed()
        && !(getLeftShooterVelocity().equals(Units.RotationsPerSecond.zero()))
        && !(getLeftShooterVelocity().equals(Units.RotationsPerSecond.zero()))
        || ignoreFlywheelSpeed);
  }

  /**
   * @return The current position of the shooter in rotations
   */
  public Measure<Angle> getShooterPosition() {
    return Units.Rotations.of(pivotMotor.getPosition().getValueAsDouble());
  }

  /**
   * @return If the shooter position is within tolerance of desired position
   */
  public boolean isShooterAtPosition(Measure<Angle> position) {
    return (Math.abs(getShooterPosition().minus(position).in(Units.Rotations)) < constShooter.AT_POSITION_TOLERANCE
        .in(Units.Rotations));

    // TODO: test if the code below works and makes our lives easier
    // return
    // (getShooterPosition().minus(position).lte(constShooter.AT_POSITION_TOLERANCE));
  }

  public void setLeftDesiredVelocity(Measure<Velocity<Angle>> desiredVelocity) {
    desiredLeftVelocity = desiredVelocity;
  }

  public void setRightDesiredVelocity(Measure<Velocity<Angle>> desiredVelocity) {
    desiredRightVelocity = desiredVelocity;
  }

  public void setDesiredVelocities(Measure<Velocity<Angle>> desiredLeftVelocity,
      Measure<Velocity<Angle>> desiredRightVelocity) {
    setLeftDesiredVelocity(desiredLeftVelocity);
    setRightDesiredVelocity(desiredRightVelocity);
  }

  public void setShooterPercentOutput(Measure<Dimensionless> speed) {
    leftMotor.set(speed.in(Units.Percent));
    rightMotor.set(speed.in(Units.Percent));
  }

  public void setLeftShooterIntakeVoltage(Measure<Voltage> voltage) {
    leftMotor.setControl(voltageRequest.withOutput(voltage.in(Units.Volts)));

  }

  public void setRightShooterIntakeVoltage(Measure<Voltage> voltage) {
    rightMotor.setControl(voltageRequest.withOutput(voltage.in(Units.Volts)));
  }

  public void setPivotVoltage(Measure<Voltage> voltage) {
    pivotMotor.setControl(voltageRequest.withOutput(voltage.in(Units.Volts)));
  }

  public void setVoltage(Measure<Voltage> leftVoltage, Measure<Voltage> rightVoltage) {
    setLeftShooterIntakeVoltage(leftVoltage);
    setRightShooterIntakeVoltage(rightVoltage);
  }

  public void setIgnoreFlywheelSpeed(boolean ignoreFlywheelSpeed) {
    this.ignoreFlywheelSpeed = ignoreFlywheelSpeed;
  }

  public void setShooterPosition(Measure<Angle> position) {
    pivotMotor.setControl(positionRequest.withPosition(position.in(Units.Rotations)));
  }

  /**
   * Sets the current angle of the pivot motor to read as the given value
   */
  public void setPivotSensorAngle(Measure<Angle> angle) {
    pivotMotor.setPosition(angle.in(Units.Rotations));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Left/Velocity RPS", getLeftShooterVelocity().in(Units.RotationsPerSecond));
    SmartDashboard.putNumber("Shooter/Left/Desired Velocity RPS", desiredLeftVelocity.in(Units.RotationsPerSecond));
    SmartDashboard.putBoolean("Shooter/Left/Up to Speed", isLeftShooterUpToSpeed());

    SmartDashboard.putNumber("Shooter/Right/Velocity RPS", getRightShooterVelocity().in(Units.RotationsPerSecond));
    SmartDashboard.putNumber("Shooter/Right/Desired Velocity RPS", desiredRightVelocity.in(Units.RotationsPerSecond));
    SmartDashboard.putBoolean("Shooter/Right/Up to Speed", isRightShooterUpToSpeed());

  }
}
