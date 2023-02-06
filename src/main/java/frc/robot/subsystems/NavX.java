// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  private AHRS gyro = new AHRS();
  public NavX() {}
  public double getAngle()
  {
    return gyro.getYaw();
  }

  /** Angle is continuous */
  public Rotation2d getRotation2d()
  {
    return gyro.getRotation2d();
  }
  public void reset()
  {
    gyro.reset();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}