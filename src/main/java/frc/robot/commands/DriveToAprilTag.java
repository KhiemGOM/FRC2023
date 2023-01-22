// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriverBase;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import static frc.robot.Constants.GridPoses.*;

public class DriveToAprilTag extends CommandBase {
  private PathPlannerTrajectory trajectory; 
  private DriverBase m_subsystem;

  public DriveToAprilTag(DriverBase __subsystem) {
    m_subsystem = __subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    Pose2d initPose = m_subsystem.getPose();
    Pose2d targetPose = LEFTGRID;
    trajectory =  PathPlanner.generatePath(
      new PathConstraints(4, 3),
      new PathPoint(initPose.getTranslation(),initPose.getRotation()),
      new PathPoint(targetPose.getTranslation(),targetPose.getRotation()));
      m_subsystem.getPathFollowCommand(trajectory, true).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //Instant command
  }
}
