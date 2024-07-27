// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapTransfer;

public class Transfer extends SubsystemBase {
  TalonFX feederMotor;
  TalonFXConfiguration feederConfig = new TalonFXConfiguration();

  /** Creates a new Transfer. */
  public Transfer() {
    feederMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN, "rio");
    feederMotor.getConfigurator().apply(feederConfig);
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
