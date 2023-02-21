// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HardwareIDs.*;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private WPI_TalonSRX grabberMotor = new WPI_TalonSRX(GRABBER_MOTOR);
  public Grabber() {}
  
  public void grab(double speed) {
    grabberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
