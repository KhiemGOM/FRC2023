// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriverBase;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.PID.*;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  NavX navX;
  DriverBase driverBase;
  PIDController PID = new PIDController(aP, aI, aD);
  public AutoBalance( DriverBase __driverBase, NavX __navX) {
    navX = __navX;
    addRequirements(navX);
    driverBase = __driverBase;
    addRequirements(driverBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PID.setTolerance(aTolerance);
    PID.setSetpoint(0);
    PID.enableContinuousInput(-180, 180);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = PID.calculate(navX.getAngle());
    driverBase.driveWithField(speed, 0, 0,navX.getRotation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint();
  }
}
