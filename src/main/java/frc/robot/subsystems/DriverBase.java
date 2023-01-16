// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.MotorIDs.*;
import static frc.robot.Constants.Measurements.*;
import static frc.robot.Constants.SingleInstance.*;

public class DriverBase extends SubsystemBase {
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(LEFT_FRONT_MOTOR);
  private WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(LEFT_BACK_MOTOR);
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RIGHT_FRONT_MOTOR);
  private WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(RIGHT_BACK_MOTOR);
  private MecanumDrive mecanum = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
  
  private MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
  CENTRE_TO_LEFT_FRONT, 
  CENTRE_TO_LEFT_BACK, 
  CENTRE_TO_RIGHT_FRONT, 
  CENTRE_TO_RIGHT_BACK);

  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, GYRO.getRotation2d(), wheelPositions);

  public DriverBase() {
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }
  public void drive (double x, double y, double rotation)
  {
    mecanum.driveCartesian(x, y, rotation);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(GYRO.getRotation2d(), new MecanumDriveWheelPositions(
      leftFrontMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096,
      rightFrontMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096,
      leftBackMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096, 
      rightBackMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096
    ));
  }
}
