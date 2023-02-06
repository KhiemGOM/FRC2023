// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private final String aprilTagFieldAbsPath = "/home/lvuser/deploy/fields/2023_FieldLayout.json";
  private PhotonPoseEstimator robotPoseEstimator;
  private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  private AprilTagFieldLayout aprilTagFieldLayout = null;
  public Camera() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(aprilTagFieldAbsPath);
    }
    catch (IOException e) {
      try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
      } catch (IOException e1) {
        // TODO Auto-generated catch block
        e1.printStackTrace();
      }
    };
    aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    robotPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
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
        return result.get().estimatedPose.toPose2d();
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
