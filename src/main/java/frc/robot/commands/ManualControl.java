// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriverBase;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.SingleInstance.*;
import static frc.robot.Constants.ButtonIDs.*;
import static frc.robot.Constants.*;

public class ManualControl extends CommandBase {
  private DriverBase m_subsystem;
  private NavX m_gyro;

  /** Creates a new ManualControl. */
  public ManualControl(DriverBase __subsystem, NavX __gyro) {
    m_subsystem = __subsystem;
    m_gyro = __gyro;

    addRequirements(m_subsystem, m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GYRO.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Manual Control Ran");
    double ySpeed = -JOYSTICK.getRawAxis(RIGHT_JOYSTICK_Y_ID); // Y joystick is inverted
    double xSpeed = -JOYSTICK.getRawAxis(RIGHT_JOYSTICK_X_ID); // X drive is inverted
    double joystickAngle = Math.toDegrees(Math.atan2(-JOYSTICK.getRawAxis(LEFT_JOYSTICK_Y_ID), JOYSTICK.getRawAxis(LEFT_JOYSTICK_X_ID)));
    double angle = 0;
    //Detect if the left joystick is being used to determine whether to set PID goal
    if (Math.abs(JOYSTICK.getRawAxis(LEFT_JOYSTICK_X_ID)) > SENSITIVITY || Math.abs(JOYSTICK.getRawAxis(LEFT_JOYSTICK_Y_ID)) > SENSITIVITY) {
      m_subsystem.setGoal(joystickAngle);
      //angle = m_subsystem.calculate(GYRO.getAngle());
    }
    m_subsystem.drive(xSpeed , ySpeed, 0);
    //m_subsystem.driveWithField(xSpeed ySpeed, angle ,m_gyro.getRotation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
