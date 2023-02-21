// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Arm;
//import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriverBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.NavX;
import static frc.robot.Constants.Measurements.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class Function{
    public static double encoderTicksToMeter(double ticks)
    {
      return ticks * WHEEL_DIAMETER * Math.PI / WHEELS_TICKS_PER_REVOLUTION;
    }
    public static double meterToEncoderTicks(double meter)
    {
      return meter * WHEELS_TICKS_PER_REVOLUTION / (WHEEL_DIAMETER * Math.PI);
    }
    public static double signof(double val)
    {
        if (val>0)
        {
          return 1;
        }
        if (val<0)
        {
          return -1;
        }
        return 0;
    }
    public static double signedSqr(double val)
    {
      return signof(val) * val * val;
    }
  }
  
  public static final double SENSITIVITY = 0.05;
  public static final double MAXINPUTWHEELMOTOR = 0.4;
  public static final double MAX_INPUT_ARM_ROTATE_MOTOR = 0.4;
  public static final double MAX_INPUT_ARM_EXTEND_MOTOR = 0.4;
  public static final double MAX_INPUT_GRABBER_MOTOR = 0.4;
  public final class HardwareIDs {
    //Motors
    public static final int LEFT_FRONT_MOTOR = 1;
    public static final int LEFT_BACK_MOTOR = 2;
    public static final int RIGHT_FRONT_MOTOR = 3;
    public static final int RIGHT_BACK_MOTOR = 4;
    public static final int ARM_ROTATE_MOTOR = 5;
    public static final int ARM_EXTEND_MOTOR = 6;
    public static final int GRABBER_MOTOR = 7;

    //Encoders
    public static final int ARM_ABSOLUTE_ENCODER = 0;
  }
  public final class ButtonIDs
  {
    public static final int RIGHT_JOYSTICK_X_ID = 2;
    public static final int RIGHT_JOYSTICK_Y_ID = 5;
    public static final int LEFT_JOYSTICK_X_ID = 0;
    public static final int LEFT_JOYSTICK_Y_ID = 1;
    public static final int ARM_ROTATE_FOWARD_ID = 1;
    public static final int ARM_ROTATE_BACKWARD_ID = 2;
    public static final int ARM_EXTEND_ID = 3;
    public static final int ARM_RETRACT_ID = 4;
    public static final int GRABBER_GRAB_ID = 5;
    public static final int GRABBER_RELEASE_ID = 6;
  }
  public final static class Measurements
  {
    public static final Translation2d CENTRE_TO_LEFT_FRONT = new Translation2d(1, 1);
    public static final Translation2d CENTRE_TO_LEFT_BACK = new Translation2d(1, 1);
    public static final Translation2d CENTRE_TO_RIGHT_FRONT = new Translation2d(1, 1);
    public static final Translation2d CENTRE_TO_RIGHT_BACK = new Translation2d(1, 1);
    public static final double WHEEL_DIAMETER = 1.5;
    public static final double WHEELS_TICKS_PER_REVOLUTION = 4096;
  }

  public final static class PID{
    //TODO: Tune PID
    //Arm Rotation PID
    public static final double rP = 0.1;
    public static final double rI = 0.1;
    public static final double rD = 0.1;
    public static final double rMaxSpeed = 3;
    public static final double rMaxAccel = 2;
    public static final double rTolerance = 5;

    //Auto Balance PID
    public static final double aP = 0.1;
    public static final double aI = 0.1;
    public static final double aD = 0.1;
    public static final double aTolerance = 5;
  }

  public static final class PathFollow
  {
    public static final double pMaxSpeed = 4;
    public static final double pMaxAccel = 3;
  }
  public final static class GridPoses
  {
    public static final Pose2d LEFTGRID = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  public final static class SingleInstance
  {
    public static NavX GYRO = new NavX();
    public static DriverBase DRIVER_BASE = new DriverBase();
    public static Arm ARM = new Arm();
    public static Grabber GRABBER = new Grabber();
    //public static Camera CAMERA = new Camera();

    public static final Joystick JOYSTICK = new Joystick(0);
  }
}
