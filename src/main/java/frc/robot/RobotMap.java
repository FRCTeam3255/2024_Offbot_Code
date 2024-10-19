package frc.robot;

// Contains all ports on our robot

public class RobotMap {
  public static class mapControllers {
    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;
    public static final int TEST_OPERATOR_USB = 5;
  }

  public static class mapDrivetrain {
    public static final String CAN_BUS_NAME = "Swerve";
    public static final int PIGEON_CAN = 0;

    // Module 0
    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    // Module 1
    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    // Module 2
    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    // Module 3
    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;
  }

  // MOTORS: 10 -> 19
  public static class mapShooter {
    public static final int SHOOTER_LEFT_MOTOR_CAN = 10;
    public static final int SHOOTER_RIGHT_MOTOR_CAN = 11;
    public static final int SHOOTER_PIVOT_MOTOR_CAN = 12;
  }

  // MOTORS: 20 -> 29
  public static class mapIntake {
    public static final int ROLLER_CAN = 20;
    public static final int NOTE_SENSOR_DIO = 2;
  }

  // MOTORS: 30 -> 39
  public static class mapElevator {
    public static final int ELEVATOR_MOTOR_CAN = 30;
    public static final int DRAINPIPE_MOTOR_CAN = 31;
    public static final int NOTE_SENSOR_DIO = 1;
  }

  // MOTORS: 40 -> 49
  public static class mapClimber {
    public static final int CLIMBER_MOTOR_CAN = 40;
  }

  // MOTORS: 50 -> 59
  public static class mapTransfer {
    public static final int TRANSFER_MOTOR_CAN = 50;
    public static final int NOTE_SENSOR_DIO = 0;
  }

}
