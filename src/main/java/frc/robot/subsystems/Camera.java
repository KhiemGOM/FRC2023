// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private final String aprilTagFieldAbsPath = "/home/lvuser/deploy/fields/2023_FieldLayout.json";
  private RobotPoseEstimator robotPoseEstimator;
  private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  private AprilTagFieldLayout aprilTagFieldLayout = null;
  public Camera() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(aprilTagFieldAbsPath);
    }
    catch (IOException e) {};
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCam));
    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camList);
  }
  //Move robot arm to april tag
  public double getVerticalAngleToAprilTag(int ID) { //Up is positive
    var list = camera.getLatestResult().getTargets();
    for (var target : list) {
      if (target.getFiducialId() == ID) {
        return target.getPitch();
      }
    }
    return 361f;
  }

  //Move driver base facing april tag
  public double getHorizontalAngleToAprilTag(int ID) { //To the right is positive
    var list = camera.getLatestResult().getTargets();
    for (var target : list) {
      if (target.getFiducialId() == ID) {
        return target.getYaw();
      }
    }
    return 361f;
  }

  public PhotonTrackedTarget getBestTarget()
  {
    return camera.getLatestResult().getBestTarget();
  }

  public Pose2d getEstimatedPose () 
  {
    var result = robotPoseEstimator.update();

    if (result.isPresent()) 
    {
        return result.get().getFirst().toPose2d();
    } 
    else 
    {
        return null;
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
