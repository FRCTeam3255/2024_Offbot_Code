// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.Target;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RobotEnableValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

    public static final double DRIVER_RUMBLE = 0.5;
    public static final double OPERATOR_RUMBLE = 0.5;
  }

  public static class constDrivetrain {
    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and
    // copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.207520;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.246826;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.239502;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.082764;

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final Measure<Distance> WHEEL_DIAMETER = Units.Inches.of(3.966);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.in(Units.Meters) * Math.PI;

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
    public static final double TRACK_WIDTH = Units.Meters.convertFrom(19.75, Units.Inches);
    // Distance between Front & Back Wheels
    public static final double WHEELBASE = Units.Meters.convertFrom(19.75, Units.Inches);

    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.KRAKEN.L3.steerGearRatio,
        WHEEL_CIRCUMFERENCE,
        SN_SwerveConstants.MK4I.KRAKEN.L3.driveGearRatio,
        SN_SwerveConstants.MK4I.KRAKEN.L3.maxSpeedMeters);

    public static final Measure<Angle> AUTO_PRELOAD_TAXI_ROTATION = Units.Degrees.of(119.62);

    public static final Measure<Angle> AT_ROTATION_TOLERANCE = Units.Degrees.of(20);

    public static final boolean DRIVE_ENABLE_CURRENT_LIMITING = true;
    public static final double DRIVE_CURRENT_THRESH = 40;
    public static final double DRIVE_CURRENT_LIMIT = 30;
    public static final double DRIVE_CURRENT_TIME_THRESH = 0.1;

    public static final boolean STEER_ENABLE_CURRENT_LIMITING = true;
    public static final double STEER_CURRENT_THRESH = 40;
    public static final double STEER_CURRENT_LIMIT = 30;
    public static final double STEER_CURRENT_TIME_THRESH = 0.1;

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

      private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(120)));
      private static final Pose3d LEFT_STAGE = new Pose3d(
          new Pose2d(4.541771411895752, 4.736017227172852, Rotation2d.fromDegrees(300)));
      private static final Pose3d CENTER_STAGE = new Pose3d(
          new Pose2d(5.554078578948975, 4.124814033508301, Rotation2d.fromDegrees(180)));
      private static final Pose3d RIGHT_STAGE = new Pose3d(
          new Pose2d(4.524875164031982, 3.488827705383301, Rotation2d.fromDegrees(60)));

      private static final Pose3d SUBWOOFER = new Pose3d(new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(0)));

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
          new Pose2d(12.0610990524292, 3.4952545166015625, Rotation2d.fromDegrees(120)));
      private static final Pose3d CENTER_STAGE = new Pose3d(
          new Pose2d(10.983105659484863, 4.096443176269531, Rotation2d.fromDegrees(0)));
      private static final Pose3d RIGHT_STAGE = new Pose3d(
          new Pose2d(12.021082878112793, 4.7371745109558105, Rotation2d.fromDegrees(240)));

      private static final Pose3d SUBWOOFER = new Pose3d(
          new Pose2d(FIELD_LENGTH.in(Units.Meters) - 1.35, 5.50, Rotation2d.fromDegrees(180)));
      private static final Pose3d SHUFFLE = new Pose3d(
          new Pose2d(FIELD_LENGTH.in(Units.Meters) - 1.2991523742675781, 7.103456497192383, Rotation2d.fromDegrees(0)));
    }
  }

  public static class constShooter {
    public static final Rotation2d SHOOTER_TO_ROBOT = new Rotation2d(Units.Degrees.of(0));
    /**
     * The position, in meters, of the center of rotation for the pivot motor
     * relative to the center of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_PIVOT = new Transform3d(
        new Translation3d(0.1778, 0, 0),
        new Rotation3d(0, 0, 180));

    public static final InvertedValue LEFT_INVERT = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue PIVOT_INVERT = InvertedValue.CounterClockwise_Positive;

    public static final double PIVOT_GEAR_RATIO = 58.5;
    public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final GravityTypeValue PIVOT_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

    // - PID -
    public static Slot0Configs LEFT_PID_SLOT_0 = new Slot0Configs();
    public static Slot1Configs LEFT_PID_SLOT_1 = new Slot1Configs();
    public static Slot0Configs RIGHT_PID_SLOT_0 = new Slot0Configs();
    public static Slot1Configs RIGHT_PID_SLOT_1 = new Slot1Configs();
    public static Slot0Configs PIVOT_PID = new Slot0Configs();
    static {
      // Left 0
      LEFT_PID_SLOT_0.kS = 0.28;
      LEFT_PID_SLOT_0.kV = 0.13;
      LEFT_PID_SLOT_0.kA = 0;
      LEFT_PID_SLOT_0.kP = 0.5;
      LEFT_PID_SLOT_0.kI = 0;
      LEFT_PID_SLOT_0.kD = 0;

      // Left 1
      LEFT_PID_SLOT_1.kS = 0.28;
      LEFT_PID_SLOT_1.kV = 0.13;
      LEFT_PID_SLOT_1.kA = 0;
      LEFT_PID_SLOT_1.kP = 0.5;
      LEFT_PID_SLOT_1.kI = 0;
      LEFT_PID_SLOT_1.kD = 0;

      // Right 0 -- Tuned!
      RIGHT_PID_SLOT_0.kS = 0.36;
      RIGHT_PID_SLOT_0.kV = 0.115;
      RIGHT_PID_SLOT_0.kA = 0;
      RIGHT_PID_SLOT_0.kP = 0.7;
      RIGHT_PID_SLOT_0.kI = 0;
      RIGHT_PID_SLOT_0.kD = 0;
      // Right 1
      RIGHT_PID_SLOT_1.kS = 0.36;
      RIGHT_PID_SLOT_1.kV = 0.115;
      RIGHT_PID_SLOT_1.kA = 0;
      RIGHT_PID_SLOT_1.kP = 0.7;
      RIGHT_PID_SLOT_1.kI = 0;
      RIGHT_PID_SLOT_1.kD = 0;
      // Pivot
      PIVOT_PID.kS = 0.4;
      PIVOT_PID.kV = 0;
      PIVOT_PID.kG = 0.53;
      PIVOT_PID.kA = 0;
      PIVOT_PID.kP = 90;
      PIVOT_PID.kD = 0;
    }

    // - Angles -
    public static final Measure<Angle> PIVOT_FORWARD_LIMIT = Units.Degrees.of(155);
    public static final Measure<Angle> PIVOT_BACKWARD_LIMIT = Units.Degrees.of(0);

    public static final Measure<Angle> PIVOT_FORWARD_INTAKE_LIMIT = PIVOT_FORWARD_LIMIT.minus(Units.Degrees.of(10));
    public static final Measure<Angle> PIVOT_BACKWARD_INTAKE_LIMIT = PIVOT_BACKWARD_LIMIT.plus(Units.Degrees.of(10));

    public static final Measure<Angle> NEUTRAL_OUT_THRESHOLD = Units.Degrees.of(65);
    public static final Measure<Angle> NONE_STATE_ANGLE = Units.Degrees.of(15);
    public static final Measure<Angle> INTAKE_SOURCE_ANGLE = Units.Degrees.of(35);

    public static final Measure<Angle> AT_POSITION_TOLERANCE = Units.Degrees.of(3);

    // - Other -
    public static final double MANUAL_PIVOT_PERCENTAGE = 0.2;

    public static final Measure<Velocity<Angle>> UP_TO_SPEED_TOLERANCE = Units.RotationsPerSecond.of(2);
    public static final Measure<Dimensionless> PREP_TO_AMP_SPEED = Units.Percent.of(0.2);
    public static final Measure<Angle> TRANSFER_TO_AMPER_ANGLE = Units.Degrees.of(110);

    public static final Measure<Time> AUTO_PREP_NONE_DELAY = Units.Seconds.of(1);

    public static final Measure<Dimensionless> INTAKE_SOURCE_SPEED = Units.Percent.of(-0.3);

    // -- Zeroing --
    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Measure<Voltage> ZEROING_VOLTAGE = Units.Volts.of(-2);

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final Measure<Velocity<Angle>> ZEROED_VELOCITY = Units.RotationsPerSecond.of(-0.01);

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

    // -- Current Limiting --
    public static final boolean PIVOT_ENABLE_CURRENT_LIMITING = true;
    public static final double PIVOT_CURRENT_THRESH = 50;
    public static final double PIVOT_CURRENT_LIMIT = 30;
    public static final double PIVOT_CURRENT_TIME_THRESH = 0.1;

    public static class ShooterPositionGroup {
      public Measure<Angle> shooterAngle;
      public Measure<Velocity<Angle>> leftVelocity, rightVelocity;
      public Measure<Distance> elevatorPosition;

      public ShooterPositionGroup(Measure<Angle> shooterAngle, Measure<Velocity<Angle>> leftVelocity,
          Measure<Velocity<Angle>> rightVelocity, Measure<Distance> elevatorPosition) {
        this.shooterAngle = shooterAngle;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.elevatorPosition = elevatorPosition;
      }
    }

    public static final ShooterPositionGroup PREP_NONE = new ShooterPositionGroup(NONE_STATE_ANGLE,
        Units.RotationsPerSecond.of(0), Units.RotationsPerSecond.of(0), Units.Meters.of(0));
    public static final ShooterPositionGroup PREP_AMP_SHOOTER = new ShooterPositionGroup(Units.Degrees.of(111),
        Units.RotationsPerSecond.of(10), Units.RotationsPerSecond.of(10), Units.Meters.of(0));
    // Amping w/ amper
    public static final ShooterPositionGroup PREP_AMP = new ShooterPositionGroup(Units.Degrees.of(99),
        Units.RotationsPerSecond.of(10), Units.RotationsPerSecond.of(10), Units.Meters.of(0.46));
    public static final ShooterPositionGroup PREP_SUB_BACKWARDS = new ShooterPositionGroup(Units.Degrees.of(111),
        Units.RotationsPerSecond.of(35), Units.RotationsPerSecond.of(35), Units.Meters.of(0.46));
    public static final ShooterPositionGroup PREP_SHUFFLE = new ShooterPositionGroup(Units.Degrees.of(42),
        Units.RotationsPerSecond.of(35), Units.RotationsPerSecond.of(35), Units.Meters.of(0));
    public static final ShooterPositionGroup PREP_SUB = new ShooterPositionGroup(Units.Degrees.of(42),
        Units.RotationsPerSecond.of(35), Units.RotationsPerSecond.of(35), Units.Meters.of(0));
    public static final ShooterPositionGroup PREP_SPIKE = new ShooterPositionGroup(Units.Degrees.of(27),
        Units.RotationsPerSecond.of(60), Units.RotationsPerSecond.of(45), Units.Meters.of(0));
    public static final ShooterPositionGroup PREP_VISION = new ShooterPositionGroup(Units.Degrees.of(-3255),
        Units.RotationsPerSecond.of(60), Units.RotationsPerSecond.of(45), Units.Meters.of(0));
    public static final ShooterPositionGroup PREP_WING = new ShooterPositionGroup(Units.Degrees.of(10.5),
        Units.RotationsPerSecond.of(60), Units.RotationsPerSecond.of(45), Units.Meters.of(0));

    public static final ShooterPositionGroup CLIMBING = new ShooterPositionGroup(Units.Degrees.of(115),
        Units.RotationsPerSecond.of(-30), Units.RotationsPerSecond.of(-30), constElevator.FORWARD_LIMIT);

    /**
     * <p>
     * Determines the necessary angle for the shooter depending on the distance from
     * the SPEAKER.
     * </p>
     * <b>KEY:</b> The distance (in meters) of the center of the shooter to the
     * SPEAKER
     * <br>
     * <br>
     * <b>VALUE:</b> The angle (in degrees) for the pivot to go up by
     * 
     */
    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();

    static {
      DISTANCE_MAP.put(1.2827, 42.0);
      DISTANCE_MAP.put(1.5875, 40.0);
      DISTANCE_MAP.put(1.8923, 34.0);
      DISTANCE_MAP.put(2.1971, 28.5);
      DISTANCE_MAP.put(2.5019, 27.5);
      DISTANCE_MAP.put(2.8067, 25.0);
      DISTANCE_MAP.put(3.1115, 23.0);
      DISTANCE_MAP.put(3.4163, 19.0);
      DISTANCE_MAP.put(3.7211, 18.5);
      DISTANCE_MAP.put(4.0259, 18.0);
      DISTANCE_MAP.put(4.3307, 17.0);
      DISTANCE_MAP.put(4.6355, 16.5);
      DISTANCE_MAP.put(4.9403, 15.0);
      DISTANCE_MAP.put(5.2451, 14.0);
      DISTANCE_MAP.put(5.5499, 13.5);
      DISTANCE_MAP.put(5.8547, 13.0);
      DISTANCE_MAP.put(6.1595, 11.5);
    }

  }

  public static class constStateMachine {
    /**
     * Returns the associated RobotState with the given TargetState
     */
    public static final Map<TargetState, RobotState> TARGET_TO_ROBOT_STATE = new HashMap<TargetState, RobotState>();

    static {
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_NONE, RobotState.PREP_NONE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_AMP_SHOOTER, RobotState.PREP_AMP_SHOOTER);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_AMP, RobotState.PREP_AMP);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SHUFFLE, RobotState.PREP_SHUFFLE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SUB_BACKWARDS, RobotState.PREP_SUB_BACKWARDS);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SPEAKER, RobotState.PREP_SPEAKER);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_VISION, RobotState.PREP_VISION);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_SPIKE, RobotState.PREP_SPIKE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_WING, RobotState.PREP_WING);
    }

    /**
     * Returns the associated shooter pivot angle and flywheel speeds for the given
     * preset TargetState
     */
    public static Map<TargetState, ShooterPositionGroup> TARGET_TO_PRESET_GROUP = new HashMap<TargetState, ShooterPositionGroup>();

    static {
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_NONE, constShooter.PREP_NONE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_AMP_SHOOTER, constShooter.PREP_AMP_SHOOTER);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SHUFFLE, constShooter.PREP_SHUFFLE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SPEAKER, constShooter.PREP_SUB);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_AMP, constShooter.PREP_AMP);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_VISION, constShooter.PREP_VISION);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SPIKE, constShooter.PREP_SPIKE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_WING, constShooter.PREP_WING);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_SUB_BACKWARDS, constShooter.PREP_SUB_BACKWARDS);
    }
  }

  public static class constClimber {
    public static final Measure<Distance> FORWARD_LIMIT = Units.Meters.of(0.6);
    public static final Measure<Distance> BACKWARD_LIMIT = Units.Meters.of(0);
    public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final double MECHANICAL_GEAR_RATIO = 12.13;
    public static final Measure<Distance> FINAL_GEAR_CIRCUMFERENCE = Units.Inches.of(1.432 * Math.PI);
    public static final double MOTOR_ROTATION_TO_METERS = MECHANICAL_GEAR_RATIO
        / FINAL_GEAR_CIRCUMFERENCE.in(Units.Meters);

    public static final Measure<Distance> AT_POSITION_TOLERANCE = Units.Meters.of(0.1);

    // -- Zeroing --
    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Measure<Voltage> ZEROING_VOLTAGE = Units.Volts.of(-2);

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
    public static final Measure<Velocity<Distance>> ZEROED_VELOCITY = Units.MetersPerSecond.of(0.05);

    /**
     * The value that the motor reports when it is at it's zeroed position. This
     * may not necessarily be 0 due to mechanical slop
     */
    public static final Measure<Distance> ZEROED_POS = Units.Meters.of(0);

    public static final Measure<Time> ZEROING_TIMEOUT = Units.Seconds.of(5);

    // -- Current Limiting --
    public static final boolean ENABLE_CURRENT_LIMITING = false;
    public static final double CURRENT_LIMIT = 30;
    public static final double CURRENT_THRESH = 60;
    public static final double CURRENT_TIME_THRESH = 0.1;
  }

  public static class constElevator {
    public static final Measure<Distance> FORWARD_LIMIT = Units.Meters.of(0.52);
    public static final Measure<Distance> BACKWARD_LIMIT = Units.Meters.of(0);
    public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue ELEVATOR_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final GravityTypeValue ELEVATOR_GRAVITY_TYPE = GravityTypeValue.Elevator_Static;
    public static final StaticFeedforwardSignValue ELEVATOR_STATIC_FFWD_SIGN = StaticFeedforwardSignValue.UseClosedLoopSign;
    public static final boolean NOTE_SENSOR_INVERT = true;
    public static final double MOTOR_ROTATION_TO_METERS = 1 / 0.0456842368;

    public static final Measure<Distance> AMP_POSITION = Units.Meters.of(0.38);
    public static final Measure<Distance> SHOOTER_ABLE_TO_MOVE_LIMIT = Units.Meters.of(0.37);

    public static final Measure<Distance> AT_POSITION_TOLERANCE = Units.Meters.of(0.05);

    public static final double DRAINPIPE_PREP_TO_AMP_SPEED = 0.2;
    public static final double DRAINPIPE_SCORE_AMP_SPEED = 1;
    public static final double DRAINPIPE_EJECTING_SPEED = -0.2;

    public static final double MANUAL_ELEVATOR_PERCENTAGE = 0.2;

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

    // -- Current Limiting --
    public static final boolean ELEVATOR_ENABLE_CURRENT_LIMITING = true;
    public static final double ELEVATOR_CURRENT_LIMIT = 30;
    public static final double ELEVATOR_CURRENT_THRESH = 50;
    public static final double ELEVATOR_CURRENT_TIME_THRESH = 0.1;

    public static final boolean DRAINPIPE_ENABLE_CURRENT_LIMITING = true;
    public static final double DRAINPIPE_CURRENT_LIMIT = 30;
    public static final double DRAINPIPE_CURRENT_THRESH = 40;
    public static final double DRAINPIPE_CURRENT_TIME_THRESH = 0.1;
  }

  public static class constIntake {
    public static final Measure<Dimensionless> INTAKING_SPEED = Units.Percent.of(1);
    public static final Measure<Dimensionless> EJECTING_SPEED = Units.Percent.of(-1);
    public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final boolean NOTE_SENSOR_INVERT = true;

    // -- Current Limiting --
    public static final boolean ENABLE_CURRENT_LIMITING = true;
    public static final double CURRENT_LIMIT = 30;
    public static final double CURRENT_THRESH = 40;
    public static final double CURRENT_TIME_THRESH = 0.1;
  }

  public static class constTransfer {
    public static final boolean NOTE_SENSOR_INVERT = true;
    public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue FEEDER_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final double INTAKING_SPEED = 0.3;
    public static final double INTAKE_SOURCE_SPEED = -0.2;
    public static final double EJECTING_SPEED = -0.3;

    public static final double PREP_TO_AMP_SPEED = 0.2;

    public static final double SHOOTING_SPEED = 0.5;

    // -- Current Limiting --
    public static final boolean ENABLE_CURRENT_LIMITING = true;
    public static final double CURRENT_LIMIT = 30;
    public static final double CURRENT_THRESH = 40;
    public static final double CURRENT_TIME_THRESH = 0.1;
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

    // The below values are accounted for in the limelight interface, NOT in code
    public static final Measure<Distance> LL_FORWARD = Units.Inches.of(11.875);
    public static final Measure<Distance> LL_RIGHT = Units.Inches.of(0);
    public static final Measure<Distance> LL_UP = Units.Inches.of(7.0);

    public static final Measure<Angle> LL_ROLL = Units.Degrees.of(0);
    public static final Measure<Angle> LL_PITCH = Units.Degrees.of(20);
    public static final Measure<Angle> LL_YAW = Units.Degrees.of(0);
  }
}
