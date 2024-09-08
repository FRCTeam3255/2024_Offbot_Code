// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  TalonFXConfiguration rollerConfiguration;

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new TalonFX(mapIntake.ROLLER_CAN, "rio");

    rollerConfiguration = new TalonFXConfiguration();

    configure();
  }

  public void configure() {
    rollerMotor.getConfigurator().apply(rollerConfiguration);
  }

  public void setIntakeRollerSpeed(Measure<Dimensionless> speed) {
    rollerMotor.set(speed.in(Units.Percent));
  }

  /**
   * Sets the rollers to neutral.
   */
  public void setRollerNeutralOutput() {
    rollerMotor.setControl(new NeutralOut());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

}
