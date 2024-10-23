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
    public static final double autoDriveP = 5;
    public static final double autoDriveI = 0;
    public static final double autoDriveD = 0;

    public static final double autoSteerP = 2.5;
    public static final double autoSteerI = 0;
    public static final double autoSteerD = 0;

    // Teleop Snapping to Rotation (Yaw)
    public static final double yawSnapP = 4;
    public static final double yawSnapI = 0;
    public static final double yawSnapD = 0;
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
