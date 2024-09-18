// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

public final class Constants {
  public static final Measure<Voltage> MAX_VOLTAGE = Units.Volts.of(12);

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
  }

  public static class constDrivetrain {
    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and
    // copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = -0.079834;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.249268;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = -0.240479;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = 0.210449;

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final double WHEEL_DIAMETER = 0.09779;
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
     */
    public static final Measure<Velocity<Distance>> DRIVE_SPEED = Units.FeetPerSecond.of(15.1);
    // Physically measured from center to center of the wheels
    // Distance between Left & Right Wheels
    public static final double TRACK_WIDTH = Units.Meters.convertFrom(23.75, Units.Inches);
    // Distance between Front & Back Wheels
    public static final double WHEELBASE = Units.Meters.convertFrom(23.75, Units.Inches);

    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.KRAKEN.L3.steerGearRatio,
        WHEEL_CIRCUMFERENCE,
        SN_SwerveConstants.MK4I.KRAKEN.L3.driveGearRatio,
        SN_SwerveConstants.MK4I.KRAKEN.L3.maxSpeedMeters);
  }

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();

    public static Measure<Distance> FIELD_LENGTH = Units.Meters.of(16.541);

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

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions. Your Speaker, Amp, Source, Left
     *         Stage, Center Stage, Right Stage, Subwoofer, Shuffle
     */
    public static Supplier<Pose3d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> new Pose3d[] { redConstants.SPEAKER_CENTER, redConstants.AMP, redConstants.SOURCE,
            redConstants.LEFT_STAGE,
            redConstants.CENTER_STAGE, redConstants.RIGHT_STAGE, redConstants.SUBWOOFER, redConstants.SHUFFLE };

      }
      return () -> new Pose3d[] { blueConstants.SPEAKER_CENTER, blueConstants.AMP, blueConstants.SOURCE,
          blueConstants.LEFT_STAGE,
          blueConstants.CENTER_STAGE, blueConstants.RIGHT_STAGE, blueConstants.SUBWOOFER, blueConstants.SHUFFLE };
    }

    /**
     * @return A list containing only the chain positions
     */
    public static List<Pose2d> getChainPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return List.of(redConstants.RIGHT_STAGE.toPose2d(), redConstants.CENTER_STAGE.toPose2d(),
            redConstants.LEFT_STAGE.toPose2d());
      } else {
        return List.of(blueConstants.RIGHT_STAGE.toPose2d(), blueConstants.CENTER_STAGE.toPose2d(),
            blueConstants.LEFT_STAGE.toPose2d());
      }
    }

    private static final class blueConstants {
      /**
       * The coordinate of the center of the blue speaker, in meters
       */
      private static final Pose3d SPEAKER_CENTER = new Pose3d(0.457 / 2, 5.557034, 2.105 - (0.133 / 2),
          new Rotation3d(0, 0, 0));
      // HEIGHT = 2.105m to the TOP of our shot. Opening is 0.133m.

      /**
       * The coordinate of the center of the blue amp, in meters.
       */
      private static final Pose3d AMP = new Pose3d(1.827, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));
      // 0.457m = The height of the AMP opening
      // 0.660m = The height between the floor and the bottom of the opening

      private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(300)));
      private static final Pose3d LEFT_STAGE = new Pose3d(
          new Pose2d(4.541771411895752, 4.736017227172852, Rotation2d.fromDegrees(120)));
      private static final Pose3d CENTER_STAGE = new Pose3d(
          new Pose2d(5.554078578948975, 4.124814033508301, Rotation2d.fromDegrees(0)));
      private static final Pose3d RIGHT_STAGE = new Pose3d(
          new Pose2d(4.524875164031982, 3.488827705383301, Rotation2d.fromDegrees(240)));

      private static final Pose3d SUBWOOFER = new Pose3d(new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(180)));

      private static final Pose3d SHUFFLE = new Pose3d(
          new Pose2d(1.2991523742675781, 7.103456497192383, Rotation2d.fromDegrees(0)));
    }

    private static final class redConstants {
      /**
       * The coordinate of the center of the red speaker, in meters
       */
      private static final Pose3d SPEAKER_CENTER = new Pose3d(FIELD_LENGTH.in(Units.Meters) - (0.457 / 2), 5.557034,
          2.105 - (0.133 / 2),
          new Rotation3d(0, 0, 0));

      /**
       * The coordinate of the center of the red amp, in meters
       */
      private static final Pose3d AMP = new Pose3d(14.706, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));

      private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(60)));
      private static final Pose3d LEFT_STAGE = new Pose3d(
          new Pose2d(12.0610990524292, 3.4952545166015625, Rotation2d.fromDegrees(300)));
      private static final Pose3d CENTER_STAGE = new Pose3d(
          new Pose2d(10.983105659484863, 4.096443176269531, Rotation2d.fromDegrees(180)));
      private static final Pose3d RIGHT_STAGE = new Pose3d(
          new Pose2d(12.021082878112793, 4.7371745109558105, Rotation2d.fromDegrees(60)));

      private static final Pose3d SUBWOOFER = new Pose3d(
          new Pose2d(FIELD_LENGTH.in(Units.Meters) - 1.35, 5.50, Rotation2d.fromDegrees(0)));

      private static final Pose3d SHUFFLE = new Pose3d(
          new Pose2d(FIELD_LENGTH.in(Units.Meters) - 1.2991523742675781, 7.103456497192383, Rotation2d.fromDegrees(0)));
    }
  }

  public static class constShooter {
    public static final Rotation2d SHOOTER_TO_ROBOT = new Rotation2d(Units.Degrees.of(180));

    public static final InvertedValue LEFT_INVERT = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue PIVOT_INVERT = InvertedValue.CounterClockwise_Positive;

    public static final double PIVOT_GEAR_RATIO = 70.2;
    public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;

    // - Angles -
    // TODO: Competitive robot should use more accurate forward and backward limits
    public static final Measure<Angle> PIVOT_FORWARD_LIMIT = Units.Degrees.of(130);
    public static final Measure<Angle> PIVOT_BACKWARD_LIMIT = Units.Degrees.of(0);

    public static final Measure<Angle> PIVOT_FORWARD_INTAKE_LIMIT = PIVOT_FORWARD_LIMIT.minus(Units.Degrees.of(10));
    public static final Measure<Angle> PIVOT_BACKWARD_INTAKE_LIMIT = PIVOT_BACKWARD_LIMIT.plus(Units.Degrees.of(10));

    public static final Measure<Angle> AT_POSITION_TOLERANCE = Units.Degrees.of(10);

    public static final double MANUAL_PIVOT_PERCENTAGE = 0.2;

    public static final Measure<Velocity<Angle>> UP_TO_SPEED_TOLERANCE = Units.RotationsPerSecond.of(5);
    public static final Measure<Dimensionless> PREP_TO_AMP_SPEED = Units.Percent.of(0.2);
    public static final Measure<Angle> TRANSFER_TO_AMPER_ANGLE = Units.Degrees.of(110);

    // -- Zeroing --
    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Measure<Voltage> ZEROING_VOLTAGE = Units.Volts.of(-1);

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final Measure<Velocity<Angle>> ZEROED_VELOCITY = Units.DegreesPerSecond.of(0.01);

    /**
     * The elapsed time required to consider the pivot motor as zeroed
     */
    public static final Measure<Time> ZEROED_TIME = Units.Seconds.of(0.25);

    /**
     * The value that the pivot reports when it is at it's zeroed position. This
     * may not necessarily be 0 due to mechanical slop
     */
    public static final Measure<Angle> ZEROED_ANGLE = Units.Degrees.of(0);

    public static final Measure<Time> ZEROING_TIMEOUT = Units.Seconds.of(3);

    public static class ShooterPositionGroup {
      public Measure<Angle> shooterAngle;
      public Measure<Velocity<Angle>> leftVelocity, rightVelocity;

      public ShooterPositionGroup(Measure<Angle> shooterAngle, Measure<Velocity<Angle>> leftVelocity,
          Measure<Velocity<Angle>> rightVelocity) {
        this.shooterAngle = shooterAngle;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
      }
    }

    public static final ShooterPositionGroup PREP_AMP_SHOOTER = new ShooterPositionGroup(Units.Degrees.of(110),
        Units.RotationsPerSecond.of(10), Units.RotationsPerSecond.of(10));
    public static final ShooterPositionGroup PREP_SHUFFLE = new ShooterPositionGroup(Units.Degrees.of(46.5),
        Units.RotationsPerSecond.of(32), Units.RotationsPerSecond.of(32));
    public static final ShooterPositionGroup PREP_SUB = new ShooterPositionGroup(Units.Degrees.of(43),
        Units.RotationsPerSecond.of(35), Units.RotationsPerSecond.of(35));
    public static final ShooterPositionGroup PREP_SPIKE = new ShooterPositionGroup(Units.Degrees.of(40),
        Units.RotationsPerSecond.of(32), Units.RotationsPerSecond.of(32));
    public static final ShooterPositionGroup PREP_WING = new ShooterPositionGroup(Units.Degrees.of(20.6995),
        Units.RotationsPerSecond.of(32), Units.RotationsPerSecond.of(32));

  }

  public static class constStateMachine {
    /**
     * Returns the associated RobotState with the given TargetState
     */
    public static final Map<TargetState, RobotState> TARGET_TO_ROBOT_STATE = new HashMap<TargetState, RobotState>();

    static {
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_AMP_SHOOTER, RobotState.PREP_AMP_SHOOTER);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SHUFFLE, RobotState.PREP_SHUFFLE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SPEAKER, RobotState.PREP_SPEAKER);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SPIKE, RobotState.PREP_SPIKE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_WING, RobotState.PREP_WING);
    }

    /**
     * Returns the associated shooter pivot angle and flywheel speeds for the given
     * preset TargetState
     */
    public static Map<TargetState, ShooterPositionGroup> TARGET_TO_PRESET_GROUP = new HashMap<TargetState, ShooterPositionGroup>();

    static {
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_AMP_SHOOTER, constShooter.PREP_AMP_SHOOTER);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SHUFFLE, constShooter.PREP_SHUFFLE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SPEAKER, constShooter.PREP_SUB);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SPIKE, constShooter.PREP_SPIKE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_WING, constShooter.PREP_WING);
    }
  }

  public static class constClimber {
    public static final Measure<Angle> FORWARD_LIMIT = Units.Rotations.of(0);
    public static final Measure<Angle> BACKWARD_LIMIT = Units.Rotations.of(0);
    public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    // TODO: Get real gear ratio values
    public static final double MECHANICAL_GEAR_RATIO = 1;
    public static final Measure<Distance> FINAL_GEAR_CIRCUMFERENCE = Units.Meters.of(1);
    public static final double MOTOR_ROTATION_TO_METERS = MECHANICAL_GEAR_RATIO
        * FINAL_GEAR_CIRCUMFERENCE.in(Units.Meters);

    // -- Zeroing --
    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Measure<Voltage> ZEROING_VOLTAGE = Units.Volts.of(-1);

    /**
     * 
     * /**
     * The elapsed time required to consider the motor as zeroed
     */
    public static final Measure<Time> ZEROED_TIME = Units.Seconds.of(1);

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final Measure<Velocity<Distance>> ZEROED_VELOCITY = Units.MetersPerSecond.of(0.2);

    /**
     * The value that the motor reports when it is at it's zeroed position. This
     * may not necessarily be 0 due to mechanical slop
     */
    public static final Measure<Distance> ZEROED_POS = Units.Meters.of(0);

    public static final Measure<Time> ZEROING_TIMEOUT = Units.Seconds.of(3);
  }

  public static class constElevator {
    public static final Measure<Angle> FORWARD_LIMIT = Units.Rotations.of(0);
    public static final Measure<Angle> BACKWARD_LIMIT = Units.Rotations.of(0);
    public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    // TODO: Get real gear ratio values
    public static final double MECHANICAL_GEAR_RATIO = 1;
    public static final Measure<Distance> FINAL_GEAR_CIRCUMFERENCE = Units.Meters.of(1);
    public static final double MOTOR_ROTATION_TO_METERS = MECHANICAL_GEAR_RATIO
        * FINAL_GEAR_CIRCUMFERENCE.in(Units.Meters);

    public static final Measure<Angle> AMP_POSITION = Units.Rotations.of(0);

    public static final Measure<Angle> AT_POSITION_TOLERANCE = Units.Rotations.of(0);

    public static final Measure<Dimensionless> DRAINPIPE_PREP_TO_AMP_SPEED = Units.Percent.of(0);
    public static final Measure<Dimensionless> DRAINPIPE_SCORE_AMP_SPEED = Units.Percent.of(0);

    public static final Measure<Time> PREP_AMP_DELAY = Units.Seconds.of(2);

    // -- Zeroing --
    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Measure<Voltage> ZEROING_VOLTAGE = Units.Volts.of(-1);

    /**
     * 
     * /**
     * The elapsed time required to consider the motor as zeroed
     */
    public static final Measure<Time> ZEROED_TIME = Units.Seconds.of(1);

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final Measure<Velocity<Distance>> ZEROED_VELOCITY = Units.MetersPerSecond.of(0.2);

    /**
     * The value that the motor reports when it is at it's zeroed position. This
     * may not necessarily be 0 due to mechanical slop
     */
    public static final Measure<Distance> ZEROED_POS = Units.Meters.of(0);

    public static final Measure<Time> ZEROING_TIMEOUT = Units.Seconds.of(3);
  }

  public static class constIntake {
    public static final Measure<Dimensionless> INTAKING_SPEED = Units.Percent.of(1);
    public static final Measure<Dimensionless> EJECTING_SPEED = Units.Percent.of(-1);
    public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
  }

  public static class constTransfer {
    public static final boolean NOTE_SENSOR_INVERT = true;
    public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue FEEDER_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final Measure<Dimensionless> INTAKING_SPEED = Units.Percent.of(0.3);
    public static final Measure<Dimensionless> EJECTING_SPEED = Units.Percent.of(-0.3);

    public static final Measure<Dimensionless> PREP_TO_AMP_SPEED = Units.Percent.of(0.2);

    public static final Measure<Dimensionless> SHOOTING_SPEED = Units.Percent.of(0.5);
  }

  public static class constLimelight {

    /**
     * <p>
     * Maximum rate of rotation before we begin rejecting pose updates
     * </p>
     */
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

    /**
     * The area that one tag (if its the only tag in the update) needs to exceed
     * before being accepted
     */
    public static final double AREA_THRESHOLD = 0.1;
  }
}
