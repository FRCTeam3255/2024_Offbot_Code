package frc.robot;

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

    // Rotational speed (degrees per second) while manually driving
    public static final Measure<Velocity<Angle>> maxTurnSpeed = Units.DegreesPerSecond.of(180);

    /**
     * <p>
     * Value to multiply with the translation velocity when slow mode is enabled
     * </p>
     * <b>Units:</b> Percentage from 0 to 1
     */
    public static final SN_DoublePreference slowModeMultiplier = new SN_DoublePreference("slowModeMultiplier", .5);

    public static final SN_DoublePreference autoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 8);
    public static final SN_DoublePreference autoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 6);

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
    public static final SN_DoublePreference autoDriveP = new SN_DoublePreference("autoDriveP", 8);
    public static final SN_DoublePreference autoDriveI = new SN_DoublePreference("autoDriveI", 0);
    public static final SN_DoublePreference autoDriveD = new SN_DoublePreference("autoDriveD", 0);

    public static final SN_DoublePreference autoSteerP = new SN_DoublePreference("autoSteerP", 2.5);
    public static final SN_DoublePreference autoSteerI = new SN_DoublePreference("autoSteerI", 0.0);
    public static final SN_DoublePreference autoSteerD = new SN_DoublePreference("autoSteerD", 0.0);

    // Teleop Snapping to Rotation (Yaw)
    public static final double yawSnapP = 3;
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

    public static final double rightShooterS = 0.4;
    public static final double rightShooterV = 0.11;
    public static final double rightShooterA = 0;
    public static final double rightShooterP = 0.4; // 0.6 ?
    public static final double rightShooterI = 0;
    public static final double rightShooterD = 0;

    public static final double pivotShooterS = 0.4;
    public static final double pivotShooterV = 0.0;
    public static final double pivotShooterG = 0.53;
    public static final double pivotShooterA = 0.0;
    public static final double pivotShooterP = 90;
    public static final double pivotShooterI = 0;
    public static final double pivotShooterD = 0;
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
