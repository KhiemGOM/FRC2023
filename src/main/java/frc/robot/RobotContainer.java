// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBalance;

import static frc.robot.Constants.SingleInstance.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import static frc.robot.Constants.ButtonIDs.*;
import static frc.robot.Constants.PID.*;
import static frc.robot.Constants.PathFollow.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //private InstantCommand m_resetOdometryToAprilTag = new InstantCommand(() -> {
  //  DRIVER_BASE.resetOdometry(CAMERA.getEstimatedPose());
  //},DRIVER_BASE, CAMERA);
  private PathPlannerTrajectory trajec = PathPlanner.loadPath("Test Path", new PathConstraints(pMaxSpeed, pMaxAccel));
  private InstantCommand m_pneumaticPush = new InstantCommand(() -> {
    PNEUMATIC.push();
  }, PNEUMATIC);
  private InstantCommand m_pneumaticPull = new InstantCommand(() -> {
    PNEUMATIC.pull();
  }, PNEUMATIC);
  private Trigger isOnRamp = new Trigger(() -> {
    return GYRO.getPitch() > aTolerance;
  });
  private JoystickButton m_pneumaticPushButton = new JoystickButton(JOYSTICK,PNEUMATIC_PUSH);
  private JoystickButton m_pneumaticPullButton = new JoystickButton(JOYSTICK,PNEUMATIC_PULL);
  private AutoBalance m_autoBalance = new AutoBalance(DRIVER_BASE, GYRO);
  public RobotContainer() {
    configureBindings();
  }
  private void configureBindings() {
    m_pneumaticPushButton.onTrue(m_pneumaticPush);
    m_pneumaticPullButton.onTrue(m_pneumaticPull);
    isOnRamp.onTrue(m_autoBalance);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //Auto Balance code
    // return new SequentialCommandGroup(
    //   new StartEndCommand(() -> 
    //     {
    //       DRIVER_BASE.driveWithField(0.4, 0, 0, GYRO.getRotation2d());
    //     }, 
    //     null, DRIVER_BASE).withTimeout(2),
    //   m_autoBalance
    // );
    return DRIVER_BASE.getPathFollowCommand(trajec, true);
  }
}
