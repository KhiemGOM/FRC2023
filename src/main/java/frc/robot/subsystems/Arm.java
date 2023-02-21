// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareIDs.*;
import static frc.robot.Constants.PID.*;

public class Arm extends SubsystemBase {
  private WPI_TalonSRX armRotateMotor = new WPI_TalonSRX(ARM_ROTATE_MOTOR);
  private WPI_TalonSRX armExtendMotor = new WPI_TalonSRX(ARM_EXTEND_MOTOR);
  private ProfiledPIDController armRotatePID = new ProfiledPIDController(rP, rI, rD, new TrapezoidProfile.Constraints(rMaxSpeed, rMaxAccel));
  private AnalogEncoder armRotateEncoder = new AnalogEncoder(ARM_ABSOLUTE_ENCODER);
  public Arm() {
    //Use getDistance() to get the angle of the arm in degrees
    armRotateEncoder.setDistancePerRotation(360);
    armRotateEncoder.reset();
    armRotatePID.setTolerance(rTolerance);
  }

  /**
   * Returns a command that rotates the arm to the specified angle
   * @param angle The angle to rotate to
   * @return A ProfiledPIDCommand that rotates the arm to the specified angle
   */
  public Command commandRotateTo (double angle)
  {
    return new ProfiledPIDCommand(armRotatePID, armRotateEncoder::getDistance, angle, this::rotate, this);
  }
  public void rotate (double speed)
  {
    armRotateMotor.set(speed);
  }
  public void extend (double speed)
  {
    armExtendMotor.set(speed);
  }
  private void rotate (double speed, State state)
  {
    rotate(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
