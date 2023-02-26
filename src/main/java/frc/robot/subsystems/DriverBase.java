// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import static frc.robot.Constants.HardwareIDs.*;
import static frc.robot.Constants.Measurements.*;
import static frc.robot.Constants.SingleInstance.*;
import static frc.robot.Constants.Function.*;
import static frc.robot.Constants.PID.*;

public class DriverBase extends SubsystemBase {
  
  //private PathPlannerTrajectory traj = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(LEFT_FRONT_MOTOR);
  private WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(LEFT_BACK_MOTOR);
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RIGHT_FRONT_MOTOR);
  private WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(RIGHT_BACK_MOTOR);
  private MecanumDrive mecanum ;
  
  private PIDController leftFrontController = new PIDController(wP, wI, wD);
  private PIDController leftBackController = new PIDController(wI, wD, wP);
  private PIDController rightFrontController = new PIDController(wI, wD, wP);
  private PIDController rightBackController = new PIDController(wI, wD, wP);
  
  private MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
  CENTRE_TO_LEFT_FRONT, 
  CENTRE_TO_LEFT_BACK, 
  CENTRE_TO_RIGHT_FRONT, 
  CENTRE_TO_RIGHT_BACK);

  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, GYRO.getRotation2d(), wheelPositions);
  //private ProfiledPIDController m_PIDController = new ProfiledPIDController(rP,rI,rD, new TrapezoidProfile.Constraints(rMaxSpeed, rMaxAccel));

  public DriverBase() {
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //Brake mode
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);

    //Disable safety
    leftFrontMotor.setSafetyEnabled(false);
    rightFrontMotor.setSafetyEnabled(false);
    leftBackMotor.setSafetyEnabled(false);
    rightBackMotor.setSafetyEnabled(false);

    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);
    mecanum = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    mecanum.setSafetyEnabled(false);

    leftFrontController.setTolerance(wTolerance);
    leftBackController.setTolerance(wTolerance);
    rightFrontController.setTolerance(wTolerance);
    rightBackController.setTolerance(wTolerance);
  }

  /**
   * Drive method for Mecanum platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is
   * independent of its angle or rotation rate.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param gyroAngle The gyro heading around the Z axis. Use this to implement field-oriented
   *     controls.
   */
  public void driveWithField (double x, double y, double rotation, Rotation2d gyroAngle)
  {
    if(notNoise(x) || notNoise(y) || notNoise(rotation))
    {
      System.out.println("x: " + x);
      mecanum.driveCartesian(x, y, rotation, gyroAngle);
    }

  }

  public void drive (MecanumDriveWheelSpeeds vel)
  {
    //TODO: Test the conversion
    //!Could be wrong
    //Current formula: (m/s) * (t/c) / d / pi * 10
    individualSet(0, meterToEncoderTicks(vel.frontLeftMetersPerSecond) * 10);
    individualSet(1, meterToEncoderTicks(vel.rearLeftMetersPerSecond) * 10);
    individualSet(2, meterToEncoderTicks(vel.frontRightMetersPerSecond) * 10);
    individualSet(3, meterToEncoderTicks(vel.rearRightMetersPerSecond) * 10);
  }
  /**
   * Drive method for Mecanum platform.
   * <p>1 is left front
   * <p>2 is left back
   * <p>3 is right front
   * <p>4 is right back
   * @param motorNumber the motor number
   * @param speed the speed of the motor
   */
  public void individualSet(int motorNumber, double speed)
  {
    MecanumDriveWheelSpeeds v = getEncoderVelocity();
    if(notNoise(speed))
    {
      switch(motorNumber)
      {
        case 0:
          leftFrontMotor.set(leftFrontController.calculate(v.frontLeftMetersPerSecond, speed));
          break;
        case 1:
          leftBackMotor.set(leftBackController.calculate(v.rearLeftMetersPerSecond, speed));
          break;
        case 2:
          rightFrontMotor.set(rightFrontController.calculate(v.frontRightMetersPerSecond, speed));
          break;
        case 3:
          rightBackMotor.set(rightBackController.calculate(v.rearRightMetersPerSecond, speed));
          break;
      }
    }
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
            //TODO: Add PID values
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

  public Command getDriveToTargetPoseCommand(Pose2d _targetPose)
  {
    Pose2d initPose = getPose();
    var trajectory =  PathPlanner.generatePath(
      new PathConstraints(4, 3),
      new PathPoint(initPose.getTranslation(),initPose.getRotation()),
      new PathPoint(_targetPose.getTranslation(),_targetPose.getRotation()));
    return getPathFollowCommand(trajectory, false);
  }

  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  public Pose2d getPoseWithCV()
  {
    if(CAMERA.getEstimatedPose() != null)
    {
      return CAMERA.getEstimatedPose();
    }
    else
    {
      return getPose();
    }
  }
  public MecanumDriveWheelSpeeds getEncoderVelocity()
  {
    return new MecanumDriveWheelSpeeds(
      encoderTicksToMeter(leftFrontMotor.getSelectedSensorVelocity()),
      encoderTicksToMeter(rightFrontMotor.getSelectedSensorVelocity()),
      encoderTicksToMeter(leftBackMotor.getSelectedSensorVelocity()),
      encoderTicksToMeter(rightBackMotor.getSelectedSensorVelocity())
    );
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
      encoderTicksToMeter(leftFrontMotor.getSelectedSensorPosition()),
      encoderTicksToMeter(rightFrontMotor.getSelectedSensorPosition()),
      encoderTicksToMeter(leftBackMotor.getSelectedSensorPosition()),
      encoderTicksToMeter(rightBackMotor.getSelectedSensorPosition())
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(GYRO.getRotation2d(), getWheelPositions());
    // if (JOYSTICK.getRawButton(1))
    //   leftFrontMotor.set(1);
    // else
    //   leftFrontMotor.set(0);
    // if (JOYSTICK.getRawButton(2))
    //   leftBackMotor.set(1);
    // else
    //   leftBackMotor.set(0);
    // if (JOYSTICK.getRawButton(3))
    //   rightFrontMotor.set(1);
    // else
    //   rightFrontMotor.set(0);
    // if (JOYSTICK.getRawButton(4))
    //   rightBackMotor.set(1);
    // else
    //   rightBackMotor.set(0);
    // if (JOYSTICK.getRawButton(1)){
    //   drive(1/Math.sqrt(2), 1/Math.sqrt(2), 0);
    // }

  // else if (JOYSTICK.getRawButton(2))
  // {
  //   drive(-1/Math.sqrt(2), -1/Math.sqrt(2), 0);
  // }
  // else
  // {
  //   drive(0,0,0);
  // }
    
    SmartDashboard.putNumber("Motor Left Front", leftFrontMotor.get());
    SmartDashboard.putNumber("Motor Left Back", leftBackMotor.get());
    SmartDashboard.putNumber("Motor Right Front", rightFrontMotor.get());
    SmartDashboard.putNumber("Motor Right Back", rightBackMotor.get());

    SmartDashboard.putNumber("Encoder Left Front", leftFrontMotor.getSelectedSensorVelocity());
  }
  
}
