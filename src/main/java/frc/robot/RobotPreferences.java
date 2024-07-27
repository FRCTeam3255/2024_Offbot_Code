package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.units.Units;

public class RobotPreferences {
  public static final class prefDrivetrain {
    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference(
        "minimumSteerSpeed", 0.01);

    // Translational speed (feet per second) while manually driving
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed",
        Constants.constDrivetrain.DRIVE_SPEED);

    // Rotational speed (degrees per second) while manually driving
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 360);

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
    public static final SN_DoublePreference yawSnapP = new SN_DoublePreference("yawSnapP", 3);
    public static final SN_DoublePreference yawSnapI = new SN_DoublePreference("yawSnapI", 0);
    public static final SN_DoublePreference yawSnapD = new SN_DoublePreference("yawSnapD", 0);
  }

  public static final class prefShooter {
    // - PID -
    public static final SN_DoublePreference leftShooterS = new SN_DoublePreference("leftShooterS", 0.4);
    public static final SN_DoublePreference leftShooterV = new SN_DoublePreference("leftShooterV", 0.12);
    public static final SN_DoublePreference leftShooterA = new SN_DoublePreference("leftShooterA", 0.1);
    public static final SN_DoublePreference leftShooterP = new SN_DoublePreference("leftShooterP", 0.6);
    public static final SN_DoublePreference leftShooterI = new SN_DoublePreference("leftShooterI", 0);
    public static final SN_DoublePreference leftShooterD = new SN_DoublePreference("leftShooterD", 0.01);

    public static final SN_DoublePreference rightShooterS = new SN_DoublePreference("rightShooterS", 0.4);
    public static final SN_DoublePreference rightShooterV = new SN_DoublePreference("rightShooterV", 0.1175);
    public static final SN_DoublePreference rightShooterA = new SN_DoublePreference("rightShooterA", 0.1);
    public static final SN_DoublePreference rightShooterP = new SN_DoublePreference("rightShooterP", 0.6);
    public static final SN_DoublePreference rightShooterI = new SN_DoublePreference("rightShooterI", 0);
    public static final SN_DoublePreference rightShooterD = new SN_DoublePreference("rightShooterD", 0.01);

    public static final SN_DoublePreference pivotShooterP = new SN_DoublePreference("pivotShooterP", 0);
    public static final SN_DoublePreference pivotShooterI = new SN_DoublePreference("pivotShooterI", 0);
    public static final SN_DoublePreference pivotShooterD = new SN_DoublePreference("pivotShooterD", 0);
  }

  public static final class prefElevator {
    public static final SN_DoublePreference elevatorShooterP = new SN_DoublePreference("elevatorShooterP", 0);
    public static final SN_DoublePreference elevatorShooterI = new SN_DoublePreference("elevatorShooterI", 0);
    public static final SN_DoublePreference elevatorShooterD = new SN_DoublePreference("elevatorShooterD", 0);
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
}
