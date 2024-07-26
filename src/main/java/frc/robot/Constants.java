// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
  /**
   * Volts
   */
  public static final double MAX_VOLTAGE = 12;

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
  }

  public static class constDrivetrain {
    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and
    // copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.322754;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.045410;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = -0.192871;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.314941;

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final double WHEEL_DIAMETER = Units.Meters.convertFrom(3.8, Units.Inches);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    // Taken from the online listing
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    /**
     * <p>
     * Theoretical maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final double THEORETICAL_MAX_DRIVE_SPEED = SN_SwerveConstants.MK4I.KRAKEN.L3.maxSpeedMeters;

    /**
     * <p>
     * Observed maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final double DRIVE_SPEED = Units.Feet.convertFrom(15.1, Units.Meters);
    // Physically measured from center to center of the wheels
    public static final double TRACK_WIDTH = Units.Meters.convertFrom(23.75, Units.Inches); // Distance between Left &
                                                                                            // Right Wheels
    public static final double WHEELBASE = Units.Meters.convertFrom(23.75, Units.Inches); // Distance between Front &
                                                                                          // Back Wheels

    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.KRAKEN.L3.steerGearRatio,
        0.09779 * Math.PI,
        SN_SwerveConstants.MK4I.KRAKEN.L3.driveGearRatio,
        SN_SwerveConstants.MK4I.KRAKEN.L3.maxSpeedMeters);
  }

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();

    /**
     * Boolean that controls when the path will be mirrored for the red
     * alliance. This will flip the path being followed to the red side of the
     * field.
     * The origin will remain on the Blue side.
     * 
     * @return If we are currently on Red alliance. Will return false if no alliance
     *         is found
     */
    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }

  public static class constShooter {
    public static final boolean LEFT_INVERT = true;
    public static final boolean RIGHT_INVERT = false;

    // - Velocities -
    public static final Measure<Velocity<Angle>> UP_TO_SPEED_TOLERANCE = Units.RotationsPerSecond.of(0.7);

    public static final Measure<Velocity<Angle>> LEFT_SPEAKER_VELOCITY = Units.RotationsPerSecond.of(60);
    public static final Measure<Velocity<Angle>> RIGHT_SPEAKER_VELOCITY = Units.RotationsPerSecond.of(45);

    // -- PRESETS --
    /**
     * Preset: Shooting while touching the subwoofer velocity
     */
    public static final Measure<Velocity<Angle>> LEFT_SUB_VELOCITY = Units.RotationsPerSecond.of(35);

    /**
     * Preset: Shooting while touching the subwoofer velocity
     */
    public static final Measure<Velocity<Angle>> RIGHT_SUB_VELOCITY = Units.RotationsPerSecond.of(35);

    public static final Measure<Velocity<Angle>> LEFT_SHUFFLE_VELOCITY = Units.RotationsPerSecond.of(32);
    public static final Measure<Velocity<Angle>> RIGHT_SHUFFLE_VELOCITY = Units.RotationsPerSecond.of(32);
  }

}
