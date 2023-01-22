// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriverBase;
import static frc.robot.Constants.SingleInstance.*;
import static frc.robot.Constants.ButtonIDs.*;

public class ManualControl extends CommandBase {
  private DriverBase m_subsystem;

  /** Creates a new ManualControl. */
  public ManualControl(DriverBase __subsystem) {
    m_subsystem = __subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double verticleSpeed = -JOYSTICK.getRawAxis(RIGHT_JOYSTICK_Y_ID); // Y joystick is inverted
    double horizontalSpeed = -JOYSTICK.getRawAxis(RIGHT_JOYSTICK_X_ID); // X drive is inverted
    double rotationSpeed = JOYSTICK.getRawAxis(LEFT_JOYSTICK_X_ID);
    m_subsystem.drive(verticleSpeed, horizontalSpeed, rotationSpeed);
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
