// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor, rightMotor, pivotMotor;
  TalonFXConfiguration leftConfig, rightConfig, pivotConfig;

  MotionMagicVelocityVoltage motionMagicRequest;
  PositionVoltage positionRequest;

  VelocityVoltage velocityRequest;
  VoltageOut voltageRequest;
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

    voltageRequest = new VoltageOut(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    motionMagicRequest = new MotionMagicVelocityVoltage(0);
    positionRequest = new PositionVoltage(0).withSlot(0);

    configure();
  }

  public void configure() {
    // -- Left Motor --
    leftConfig.MotorOutput.Inverted = constShooter.LEFT_INVERT;
    leftConfig.Slot0.kV = prefShooter.leftShooterV;
    leftConfig.Slot0.kS = prefShooter.leftShooterS;
    leftConfig.Slot0.kA = prefShooter.leftShooterA;
    leftConfig.Slot0.kP = prefShooter.leftShooterP;
    leftConfig.Slot0.kI = prefShooter.leftShooterI;
    leftConfig.Slot0.kD = prefShooter.leftShooterD;

    leftConfig.MotionMagic.MotionMagicAcceleration = 400;
    leftConfig.MotionMagic.MotionMagicJerk = 4000;
    leftMotor.getConfigurator().apply(leftConfig);

    // -- Right Motor --
    rightConfig.MotorOutput.Inverted = constShooter.RIGHT_INVERT;
    rightConfig.Slot0.kV = prefShooter.rightShooterV;
    rightConfig.Slot0.kS = prefShooter.rightShooterS;
    rightConfig.Slot0.kA = prefShooter.rightShooterA;
    rightConfig.Slot0.kP = prefShooter.rightShooterP;
    rightConfig.Slot0.kI = prefShooter.rightShooterI;
    rightConfig.Slot0.kD = prefShooter.rightShooterD;

    rightConfig.MotionMagic.MotionMagicAcceleration = 400;
    rightConfig.MotionMagic.MotionMagicJerk = 4000;
    rightMotor.getConfigurator().apply(rightConfig);

    // -- Pivot Motor --
    pivotConfig.Feedback.SensorToMechanismRatio = constShooter.PIVOT_GEAR_RATIO;
    pivotConfig.MotorOutput.Inverted = constShooter.PIVOT_INVERT;
    pivotConfig.MotorOutput.NeutralMode = constShooter.PIVOT_NEUTRAL_MODE;
    pivotConfig.Slot0.kS = prefShooter.pivotShooterS;
    pivotConfig.Slot0.kV = prefShooter.pivotShooterV;
    pivotConfig.Slot0.kG = prefShooter.pivotShooterG;
    pivotConfig.Slot0.kA = prefShooter.pivotShooterA;
    pivotConfig.Slot0.kP = prefShooter.pivotShooterP;
    pivotConfig.Slot0.kI = prefShooter.pivotShooterI;
    pivotConfig.Slot0.kD = prefShooter.pivotShooterD;
    pivotConfig.Slot0.GravityType = constShooter.PIVOT_GRAVITY_TYPE;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constShooter.PIVOT_FORWARD_LIMIT.in(Units.Rotations);

    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constShooter.PIVOT_BACKWARD_LIMIT.in(Units.Rotations);
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void setPivotSoftwareLimits(boolean reverse, boolean forward) {
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    pivotMotor.getConfigurator().apply(pivotConfig);
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
   * Sets all of the flywheel motors to neutral.
   */
  public void setShootingNeutralOutput() {
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  public void setPivotNeutralOutput() {
    pivotMotor.setControl(new NeutralOut());
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

  /**
   * Sets the desired speeds for the flywheels, as well as the desired pivot
   * position for the shooter, and attempts to get up to speed.
   * 
   * @param shooterPositionGroup
   */
  public void setDesiredPosition(ShooterPositionGroup shooterPositionGroup) {
    setDesiredVelocities(shooterPositionGroup.leftVelocity, shooterPositionGroup.rightVelocity);
    setPivotPosition(shooterPositionGroup.shooterAngle);
    getUpToSpeed();
  }

  public void setShooterPercentOutput(Measure<Dimensionless> speed) {
    leftMotor.set(speed.in(Units.Percent));
    rightMotor.set(speed.in(Units.Percent));
  }

  public void setPivotPercentOutput(Measure<Dimensionless> speed) {
    pivotMotor.set(speed.in(Units.Percent));
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

  public void setPivotPosition(Measure<Angle> position) {
    pivotMotor.setControl(positionRequest.withPosition(position.in(Units.Rotations)));
  }

  /**
   * Sets the current angle of the pivot motor to read as the given value
   */
  public void setPivotSensorAngle(Measure<Angle> angle) {
    pivotMotor.setPosition(angle.in(Units.Rotations));
  }

  /**
   * Calculates the desired angle needed to lock onto the speaker.
   * 
   * @param robotPose  The current pose of the robot
   * @param fieldPoses The poses of the field elements, matching your alliance
   *                   color
   * 
   * @return The desired angle required to lock onto the speaker
   */
  public Measure<Angle> getDesiredAngleToLock(Pose2d robotPose, Pose3d[] fieldPoses) {

    Pose3d targetPose = fieldPoses[0];

    // Get the pitch pose (field relative)
    Pose3d pitchPose = new Pose3d(robotPose).transformBy(constShooter.ROBOT_TO_PIVOT);

    // Get distances from the pitch pose to the target pose and then calculate the
    // required angle
    // Theres probably a WPILib method for this
    double distX = Math.abs(targetPose.getX() - pitchPose.getX());
    double distY = Math.abs(targetPose.getY() - pitchPose.getY());
    double distanceFromSpeaker = Math.hypot(distX, distY);
    SmartDashboard.putNumber("DISTANCE_FROM_SPEAKER", distanceFromSpeaker);
    Measure<Angle> desiredLockingAngle = Units.Degrees.of(constShooter.DISTANCE_MAP.get(distanceFromSpeaker));

    return desiredLockingAngle;
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

    SmartDashboard.putNumber("Shooter/Pivot", getShooterPosition().in(Units.Degrees));
    SmartDashboard.putNumber("Shooter/Pivot Velocity", pivotMotor.getVelocity().getValueAsDouble());

  }
}
