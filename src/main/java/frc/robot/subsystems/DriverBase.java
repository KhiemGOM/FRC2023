// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import static frc.robot.Constants.MotorIDs.*;
import static frc.robot.Constants.Measurements.*;
import static frc.robot.Constants.SingleInstance.*;
import static frc.robot.Constants.PID.*;

public class DriverBase extends SubsystemBase {
  
  //private PathPlannerTrajectory traj = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(LEFT_FRONT_MOTOR);
  private WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(LEFT_BACK_MOTOR);
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RIGHT_FRONT_MOTOR);
  private WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(RIGHT_BACK_MOTOR);
  private MecanumDrive mecanum ;
  
  private MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
  CENTRE_TO_LEFT_FRONT, 
  CENTRE_TO_LEFT_BACK, 
  CENTRE_TO_RIGHT_FRONT, 
  CENTRE_TO_RIGHT_BACK);

  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, GYRO.getRotation2d(), wheelPositions);
  private ProfiledPIDController m_PIDController = new ProfiledPIDController(rP,rI,rD, new TrapezoidProfile.Constraints(rMaxSpeed, rMaxAccel));

  public DriverBase() {
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //Disable safety
    leftFrontMotor.setSafetyEnabled(false);
    rightFrontMotor.setSafetyEnabled(false);
    leftBackMotor.setSafetyEnabled(false);
    rightBackMotor.setSafetyEnabled(false);

    mecanum = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    mecanum.setSafetyEnabled(false);

    m_PIDController.enableContinuousInput(-180, 180);
  }

  public void driveWithField (double x, double y, double rotation, Rotation2d gyroAngle)
  {
    mecanum.driveCartesian(x, y, rotation, gyroAngle);
  }

  public ProfiledPIDController getRotatePIDController ()
  {
    return m_PIDController;
  }
  public void setGoal(double goal)
  {
    m_PIDController.setGoal(goal);
  }
  public double calculate (double currentAngle)
  {
    return m_PIDController.calculate(currentAngle);
  }

  public void drive (double x, double y, double rotation)
  {
    System.out.println("Drive() Ran");
    mecanum.driveCartesian(x, y, rotation);
  }

  public void drive (MecanumDriveWheelSpeeds vel)
  {
    leftFrontMotor.set(vel.frontLeftMetersPerSecond);
    leftBackMotor.set(vel.rearLeftMetersPerSecond);
    rightFrontMotor.set(vel.frontRightMetersPerSecond);
    rightBackMotor.set(vel.rearRightMetersPerSecond);
  }

  public Command getPathFollowCommand (PathPlannerTrajectory _traj, boolean isFirstPath)
  {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(_traj.getInitialHolonomicPose());
          }
        },DRIVER_BASE),
        new PPMecanumControllerCommand(
            _traj, 
            this::getPose, // Pose supplier
            this.kinematics, // MecanumDriveKinematics
            new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            3.0, // Max wheel velocity meters per second
            this::drive, // MecanumDriveWheelSpeeds consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }
  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d _initialPose)
  {
    resetEncoders();
    odometry.resetPosition(GYRO.getRotation2d(), getWheelPositions(), _initialPose);
  }

  public void resetEncoders()
  {
    leftFrontMotor.setSelectedSensorPosition(0);
    leftBackMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
  }

  private MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      leftFrontMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096,
      rightFrontMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096,
      leftBackMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096, 
      rightBackMotor.getSelectedSensorPosition() * WHEEL_DIAMETER * Math.PI / 4096
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(GYRO.getRotation2d(), getWheelPositions());
  }
  
}
