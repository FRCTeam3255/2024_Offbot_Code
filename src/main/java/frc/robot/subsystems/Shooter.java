// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.RobotMap.mapShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor, rightMotor, pivotMotor;
  TalonFXConfiguration leftConfig, rightConfig, pivotConfig;

  MotionMagicVelocityVoltage motionMagicRequest;
  PositionVoltage positionRequest;
  MotionMagicVoltage motionMagicPivotRequest;
  MusicTone musicRequest;

  VelocityVoltage velocityRequest;
  VoltageOut voltageRequest;
  private boolean ignoreFlywheelSpeed = false;
  private Measure<Velocity<Angle>> desiredLeftVelocity = Units.RotationsPerSecond.of(0);
  private Measure<Velocity<Angle>> desiredRightVelocity = Units.RotationsPerSecond.of(0);
  private Measure<Angle> lastDesiredPivotAngle = Units.Degrees.of(-3255);

  int currentRightSlot = 0;
  int currentLeftSlot = 0;

  public static boolean attemptingZeroing = false;
  public static boolean hasZeroed = false;

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
    motionMagicPivotRequest = new MotionMagicVoltage(0);
    positionRequest = new PositionVoltage(0).withSlot(0);
    musicRequest = new MusicTone(0);

    configure();
  }

  public void configure() {
    // -- Left Motor --
    leftConfig.MotorOutput.Inverted = constShooter.LEFT_INVERT;
    leftConfig.Slot0 = constShooter.LEFT_PID_SLOT_0_FAST;
    leftConfig.Slot1 = constShooter.LEFT_PID_SLOT_1_SLOW;

    leftConfig.MotionMagic.MotionMagicCruiseVelocity = 60;
    leftConfig.MotionMagic.MotionMagicAcceleration = 600;
    leftConfig.MotionMagic.MotionMagicJerk = 6000;
    leftMotor.getConfigurator().apply(leftConfig);

    // -- Right Motor --
    rightConfig.MotorOutput.Inverted = constShooter.RIGHT_INVERT;
    rightConfig.Slot0 = constShooter.RIGHT_PID_SLOT_0_FAST;
    rightConfig.Slot1 = constShooter.RIGHT_PID_SLOT_1_SLOW;

    rightConfig.MotionMagic.MotionMagicCruiseVelocity = 60;
    rightConfig.MotionMagic.MotionMagicAcceleration = 600;
    rightConfig.MotionMagic.MotionMagicJerk = 6000;
    rightMotor.getConfigurator().apply(rightConfig);

    // -- Pivot Motor --
    pivotConfig.Feedback.SensorToMechanismRatio = constShooter.PIVOT_GEAR_RATIO;
    pivotConfig.MotorOutput.Inverted = constShooter.PIVOT_INVERT;
    pivotConfig.MotorOutput.NeutralMode = constShooter.PIVOT_NEUTRAL_MODE;
    pivotConfig.Slot0 = constShooter.PIVOT_PID;
    pivotConfig.Audio.AllowMusicDurDisable = constShooter.PIVOT_SINGS_IN_DISABLE;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constShooter.PIVOT_FORWARD_LIMIT.in(Units.Rotations);

    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constShooter.PIVOT_BACKWARD_LIMIT.in(Units.Rotations);

    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 160;
    pivotConfig.MotionMagic.MotionMagicJerk = 1600;

    // - Current Limits -
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = constShooter.PIVOT_ENABLE_CURRENT_LIMITING;
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = constShooter.PIVOT_CURRENT_THRESH;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = constShooter.PIVOT_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = constShooter.PIVOT_CURRENT_TIME_THRESH;

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
      currentLeftSlot = (desiredLeftVelocity.lte(constShooter.LEFT_SLOT_1_THRESH)) ? 1 : 0;
      currentRightSlot = (desiredRightVelocity.lte(constShooter.RIGHT_SLOT_1_THRESH)) ? 1 : 0;

      leftMotor.setControl(
          motionMagicRequest.withVelocity(desiredLeftVelocity.in(Units.RotationsPerSecond)).withSlot(currentLeftSlot));
      rightMotor
          .setControl(motionMagicRequest.withVelocity(desiredRightVelocity.in(Units.RotationsPerSecond))
              .withSlot(currentRightSlot));
    }
  }

  /**
   * Sets all of the flywheel motors to neutral.
   */
  public void setShootingNeutralOutput() {
    setDesiredVelocities(Units.RotationsPerSecond.zero(), Units.RotationsPerSecond.zero());
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

  public Measure<Velocity<Angle>> getPivotRotorVelocity() {
    return Units.RotationsPerSecond.of(pivotMotor.getRotorVelocity().getValueAsDouble());
  }

  public Measure<Voltage> getPivotCurrent() {
    return Units.Volts.of(pivotMotor.getStatorCurrent().getValueAsDouble());
  }

  /**
   * @return If the left shooter motor is at its desired velocity
   */
  public boolean isLeftShooterUpToSpeed() {
    if (desiredLeftVelocity.baseUnitMagnitude() == 0) {
      return false;
    }
    return (desiredLeftVelocity.minus(getLeftShooterVelocity())).lte(constShooter.UP_TO_SPEED_TOLERANCE);
  }

  /**
   * @return If the right shooter motor is at its desired velocity
   */
  public boolean isRightShooterUpToSpeed() {
    if (desiredRightVelocity.baseUnitMagnitude() == 0) {
      return false;
    }

    return (desiredRightVelocity.minus(getRightShooterVelocity())).lte(constShooter.UP_TO_SPEED_TOLERANCE);
  }

  /**
   * @return If both motors have a non-zero desired velocity and are at their
   *         desired velocities
   */
  public boolean areBothShootersUpToSpeed() {
    return (isLeftShooterUpToSpeed() && isRightShooterUpToSpeed() || ignoreFlywheelSpeed);
  }

  /**
   * @return The current position of the shooter in rotations
   */
  public Measure<Angle> getShooterPosition() {
    return Units.Rotations.of(pivotMotor.getPosition().getValueAsDouble());
  }

  public boolean isSafeToMoveElevator() {
    return getShooterPosition().lte(constShooter.NEUTRAL_OUT_THRESHOLD);
  }

  /**
   * @return If the shooter position is within tolerance of desired position
   */
  public boolean isShooterAtPosition(Measure<Angle> position) {
    if (lastDesiredPivotAngle.in(Units.Degrees) == -3255) {
      return false;
    }
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
    lastDesiredPivotAngle = position;
    pivotMotor.setControl(motionMagicPivotRequest.withPosition(position.in(Units.Rotations)));
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

  public boolean readyToShoot() {
    return isLeftShooterUpToSpeed() && isRightShooterUpToSpeed() && isShooterAtPosition(lastDesiredPivotAngle);
  }

  // -- SysID
  final SysIdRoutine leftFlywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Units.Volts.of(7), // Reduce dynamic step voltage to 4 to prevent brownout
          null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> leftMotor.setControl(voltageRequest.withOutput(volts.in(Units.Volts))),
          null,
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return leftFlywheelSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return leftFlywheelSysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Left/Velocity RPS", getLeftShooterVelocity().in(Units.RotationsPerSecond));
    SmartDashboard.putNumber("Shooter/Left/Desired Velocity RPS", desiredLeftVelocity.in(Units.RotationsPerSecond));
    SmartDashboard.putBoolean("Shooter/Left/Up to Speed", isLeftShooterUpToSpeed());
    SmartDashboard.putNumber("Shooter/Left/PID Slot", currentLeftSlot);
    SmartDashboard.putNumber("Shooter/Left/Stator Current", leftMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Left/Supply Current", leftMotor.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Shooter/Right/Velocity RPS", getRightShooterVelocity().in(Units.RotationsPerSecond));
    SmartDashboard.putNumber("Shooter/Right/Desired Velocity RPS", desiredRightVelocity.in(Units.RotationsPerSecond));
    SmartDashboard.putBoolean("Shooter/Right/Up to Speed", isRightShooterUpToSpeed());
    SmartDashboard.putNumber("Shooter/Right/PID Slot", currentRightSlot);
    SmartDashboard.putNumber("Shooter/Right/Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Right/Supply Current", rightMotor.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Shooter/Pivot/Position", getShooterPosition().in(Units.Degrees));
    SmartDashboard.putNumber("Shooter/Pivot/Last Desired Angle", lastDesiredPivotAngle.in(Units.Degrees));
    SmartDashboard.putBoolean("Shooter/Pivot/At Desired Position", isShooterAtPosition(lastDesiredPivotAngle));
    SmartDashboard.putNumber("Shooter/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Pivot/Rotor Velocity", pivotMotor.getRotorVelocity().getValueAsDouble());

    SmartDashboard.putBoolean("Shooter/Safe to Move Elevator", isSafeToMoveElevator());
    SmartDashboard.putBoolean("Shooter/Ready to Shoot", readyToShoot());
    SmartDashboard.putNumber("Shooter/Last Desired Pivot Angle", lastDesiredPivotAngle.in(Units.Degrees));

    SmartDashboard.putBoolean("Zeroing/Pivot/Attempting Zeroing", attemptingZeroing);
    SmartDashboard.putBoolean("Zeroing/Pivot/Has Zeroed", hasZeroed);
  }
}
