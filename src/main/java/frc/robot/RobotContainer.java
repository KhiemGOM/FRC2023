// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.SingleInstance.*;
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
  
  private InstantCommand m_pneumaticPush = new InstantCommand(() -> {
    PNEUMATIC.push();
  }, PNEUMATIC);
  private InstantCommand m_pneumaticPull = new InstantCommand(() -> {
    PNEUMATIC.pull();
  }, PNEUMATIC);
  private JoystickButton m_pneumaticPushButton = new JoystickButton(JOYSTICK,PNEUMATIC_PUSH);
  private JoystickButton m_pneumaticPullButton = new JoystickButton(JOYSTICK,PNEUMATIC_PULL);
  public RobotContainer() {
    configureBindings();
  }
  private void configureBindings() {
    m_pneumaticPushButton.onTrue(m_pneumaticPush);
    m_pneumaticPullButton.onTrue(m_pneumaticPull);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
