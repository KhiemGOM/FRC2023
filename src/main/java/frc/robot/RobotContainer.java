// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBalance;

import static frc.robot.Constants.SingleInstance.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import static frc.robot.Constants.PID.*;
import static frc.robot.Constants.PathFollow.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ButtonIDs.*;


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
  private Trigger isOnRamp = new Trigger(() -> {
    return GYRO.getPitch() > aTolerance;
  });
  private JoystickButton armRotateFowardButton= new JoystickButton(JOYSTICK,ARM_ROTATE_FOWARD_ID);
  private JoystickButton armRotateBackwardButton= new JoystickButton(JOYSTICK,ARM_ROTATE_BACKWARD_ID);
  private JoystickButton armExtendButton = new JoystickButton(JOYSTICK,ARM_EXTEND_ID);
  private JoystickButton armRetractButton = new JoystickButton(JOYSTICK,ARM_RETRACT_ID);
  private JoystickButton grabberGrabButton = new JoystickButton(JOYSTICK,GRABBER_GRAB_ID);
  private JoystickButton grabberReleaseButton = new JoystickButton(JOYSTICK,GRABBER_RELEASE_ID);

  private AutoBalance m_autoBalance = new AutoBalance(DRIVER_BASE, GYRO);
  public RobotContainer() {
    configureBindings();
    SmartDashboard.putNumber("Speed Test", 500);
  }
  private void configureBindings() {
    isOnRamp.onTrue(m_autoBalance);

    armRotateBackwardButton.onTrue(new StartEndCommand(() -> 
    {
      ARM.rotate(-MAX_INPUT_ARM_ROTATE_MOTOR);
    },() -> 
    {
      ARM.rotate(0);
    }, ARM));

    armRotateFowardButton.onTrue(new StartEndCommand(() -> 
    {
      ARM.rotate(MAX_INPUT_ARM_ROTATE_MOTOR);
    },() -> 
    {
      ARM.rotate(0);
    }, ARM));

    armExtendButton.onTrue(new StartEndCommand(() -> 
    {
      ARM.extend(MAX_INPUT_ARM_EXTEND_MOTOR);
    },() -> 
    {
      ARM.extend(0);
    }, ARM));
    
    armRetractButton.onTrue(new StartEndCommand(() -> 
    {
      ARM.extend(-MAX_INPUT_ARM_EXTEND_MOTOR);
    },() -> 
    {
      ARM.extend(0);
    }, ARM));

    grabberGrabButton.onTrue(new StartEndCommand(() -> 
    {
      GRABBER.grab(MAX_INPUT_GRABBER_MOTOR);
    },() -> 
    {
      GRABBER.grab(0);
    }, GRABBER));

    grabberReleaseButton.onTrue(new StartEndCommand(() -> 
    {
      GRABBER.grab(-MAX_INPUT_GRABBER_MOTOR);
    },() -> 
    {
      GRABBER.grab(0);
    }, GRABBER));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //Auto Balance code
    //return m_autoBalance;
    return new RunCommand(()->
    {
      DRIVER_BASE.driveWithField(0.4, 0, 0, GYRO.getRotation2d());
    }, DRIVER_BASE).withTimeout(2);
    //);
    //return DRIVER_BASE.getPathFollowCommand(trajec, true);
    // return new PIDCommand(
    //   new PIDController(1, 0, 0), 
    //   ()-> {return DRIVER_BASE.getEncoderVelocity().frontLeftMetersPerSecond;}, 
    //   ()-> {return SmartDashboard.getNumber("Speed Test", 500);}, 
    //   (double speed) -> {DRIVER_BASE.individualSet(1,speed);}, 
    //   DRIVER_BASE);
  }
}
