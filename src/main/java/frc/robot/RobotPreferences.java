package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class RobotPreferences {
  public static final class prefDrivetrain {
    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference(
        "minimumSteerSpeed", 0.01);

    // Translational speed (Meters per second) while manually driving
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed",
        Constants.constDrivetrain.DRIVE_SPEED.in(Units.MetersPerSecond));

    // Rotational speed (degrees per second) while MANUALLY driving
    public static final Measure<Velocity<Angle>> maxManualTurnSpeed = Units.DegreesPerSecond.of(30);
    public static final Measure<Velocity<Angle>> maxTurnSpeed = Units.DegreesPerSecond.of(520);

    /**
     * <p>
     * Value to multiply with the translation velocity when slow mode is enabled
     * </p>
     * <b>Units:</b> Percentage from 0 to 1
     */
    public static final SN_DoublePreference slowModeMultiplier = new SN_DoublePreference("slowModeMultiplier", .5);

    public static final SN_DoublePreference autoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 6);
    public static final SN_DoublePreference autoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 4);

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final SN_DoublePreference measurementStdDevsPosition = new SN_DoublePreference(
        "measurementStdDevsPosition", 0.05);

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final SN_DoublePreference measurementStdDevsHeading = new SN_DoublePreference(
        "measurementStdDevsHeading", Units.Radians.convertFrom(5, Units.Degrees));

    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.18);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 100);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 0.14414076246334312);

    public static final SN_DoublePreference driveKs = new SN_DoublePreference("driveKs", 0);
    public static final SN_DoublePreference driveKa = new SN_DoublePreference("driveKa", 0);
    public static final SN_DoublePreference driveKv = new SN_DoublePreference("driveKv", (1 / driveSpeed.getValue()));

    // This PID is implemented on the Drivetrain subsystem
    public static final SN_DoublePreference autoDriveP = new SN_DoublePreference("autoDriveP", 4);
    public static final SN_DoublePreference autoDriveI = new SN_DoublePreference("autoDriveI", 0);
    public static final SN_DoublePreference autoDriveD = new SN_DoublePreference("autoDriveD", 0);

    public static final SN_DoublePreference autoSteerP = new SN_DoublePreference("autoSteerP", 3.255);
    public static final SN_DoublePreference autoSteerI = new SN_DoublePreference("autoSteerI", 0.0);
    public static final SN_DoublePreference autoSteerD = new SN_DoublePreference("autoSteerD", 0.0);

    // Teleop Snapping to Rotation (Yaw)
    public static final double yawSnapP = 4;
    public static final double yawSnapI = 0;
    public static final double yawSnapD = 0;
  }

  public static final class prefShooter {
    // - PID -

    public static final double leftShooterS = 0.28;
    public static final double leftShooterV = 0.13;
    public static final double leftShooterA = 0.0;
    public static final double leftShooterP = 0.5; // 0.75 ?
    public static final double leftShooterI = 0;
    public static final double leftShooterD = 0.0;

    public static Slot0Configs leftPIDSlot0 = new Slot0Configs();
    public static Slot1Configs leftPIDSlot1 = new Slot1Configs();

    public static Slot0Configs rightPIDSlot0 = new Slot0Configs();
    public static Slot1Configs rightPIDSlot1 = new Slot1Configs();

    public static Slot0Configs pivotPID = new Slot0Configs();

    static {
      // Left 0
      leftPIDSlot0.kS = 0.28;
      leftPIDSlot0.kV = 0.13;
      leftPIDSlot0.kA = 0;
      leftPIDSlot0.kP = 0.5;
      leftPIDSlot0.kI = 0;
      leftPIDSlot0.kD = 0;

      // Left 1
      leftPIDSlot1.kS = 0.28;
      leftPIDSlot1.kV = 0.13;
      leftPIDSlot1.kA = 0;
      leftPIDSlot1.kP = 0.5;
      leftPIDSlot1.kI = 0;
      leftPIDSlot1.kD = 0;

      // Right 0 -- Tuned!
      rightPIDSlot0.kS = 0.36;
      rightPIDSlot0.kV = 0.115;
      rightPIDSlot0.kA = 0;
      rightPIDSlot0.kP = 0.7;
      rightPIDSlot0.kI = 0;
      rightPIDSlot0.kD = 0;
      // Right 1
      rightPIDSlot1.kS = 0.36;
      rightPIDSlot1.kV = 0.115;
      rightPIDSlot1.kA = 0;
      rightPIDSlot1.kP = 0.7;
      rightPIDSlot1.kI = 0;
      rightPIDSlot1.kD = 0;
      // Pivot
      pivotPID.kS = 0.4;
      pivotPID.kV = 0;
      pivotPID.kG = 0.53;
      pivotPID.kA = 0;
      pivotPID.kP = 90;
      pivotPID.kD = 0;
    }
  }

  public static final class prefElevator {
    public static final double elevatorG = 0.3;
    public static final double elevatorS = 0.4;
    public static final double elevatorP = 25;
    public static final double elevatorI = 0;
    public static final double elevatorD = 0;
  }

  public static final class prefVision {
    /**
     * <p>
     * Pose estimator standard deviation for vision data using Multi-tag
     * <p>
     * <b>Units:</b> Meters
     */
    public static final SN_DoublePreference multiTagStdDevsPosition = new SN_DoublePreference(
        "multiTagStdDevsPosition", 0.7);

    /**
     * <p>
     * Pose estimator standard deviation for vision data using Multi-Tag
     * </p>
     * <b>Units:</b> Radians
     */
    public static final SN_DoublePreference multiTagStdDevsHeading = new SN_DoublePreference(
        "multiTagStdDevsHeading", 9999999);
  }

  public static final class prefIntake {
  }

  public static final class prefTransfer {
  }
}
