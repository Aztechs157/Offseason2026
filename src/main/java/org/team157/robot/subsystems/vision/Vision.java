// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot.subsystems.vision;

import static org.team157.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.subsystems.drive.Drive;
import org.team157.robot.subsystems.vision.VisionIO.PoseObservationType;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean isBlueAlliance = true;

  // Targeting values, updated every loop by the default command
  private Rotation2d driveAngleToFaceTarget = Rotation2d.kZero;
  private double distanceToTarget = 0;
  private boolean isAimed = false;
  private boolean isUnderTrench = false;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /**
   * Keeps the targeting values (angle and distance to the current target) up to date. Also runs
   * while disabled, so targeting can be checked in AdvantageScope by pushing the robot around.
   *
   * @param drivetrain The drivetrain, used for the robot's current pose.
   * @return a {@link Command} updating the targeting values every loop.
   */
  public Command setDefault(Drive drivetrain) {
    return run(() -> {
          updateAlliance();
          Pose2d robotPose = drivetrain.getPose();
          setTargetParams(getDesiredPose(robotPose), robotPose);
        })
        .ignoringDisable(true);
  }

  /**
   * Gets the point the robot should shoot at, based on the current alliance and the robot's
   * location on the field (the hub, or a passing point when in the neutral zone).
   *
   * @return the target point on the field, as a Pose2d.
   */
  public Pose2d getDesiredPose(Pose2d robotPose) {
    return FieldConstants.positionDetails.getTargetPose2d(robotPose, isBlueAlliance);
  }

  public void updateAlliance() {
    isBlueAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;
  }

  /**
   * Calculates the heading that points the back of the robot (where the dumper shoots from) at the
   * target, and the distance from the shooter to the target.
   *
   * @param targetPose the target Pose2d to aim at
   * @param robotPose the current Pose2d of the robot
   */
  public void setTargetParams(Pose2d targetPose, Pose2d robotPose) {
    // The shooter is on the robot's centerline, so facing the back of the robot at the target from
    // the robot's center also lines up the shooter.
    Rotation2d angleFromRobotToTarget =
        targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle();
    driveAngleToFaceTarget = angleFromRobotToTarget.plus(Rotation2d.k180deg);

    Pose2d shooterPose = robotPose.transformBy(VisionConstants.ROBOT_TO_SHOOTER);
    distanceToTarget = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

    isAimed =
        Math.abs(robotPose.getRotation().minus(driveAngleToFaceTarget).getRadians())
            < VisionConstants.AIM_TOLERANCE.getRadians();

    // The hood is at the back of the robot, so check where the shooter is rather than the center
    isUnderTrench = FieldConstants.positionDetails.isUnderTrench(shooterPose);
    Logger.recordOutput("Targeting/Is Under Trench", isUnderTrench);

    Logger.recordOutput("Targeting/Target Pose", targetPose);
    Logger.recordOutput("Targeting/Shooter Pose", shooterPose);
    Logger.recordOutput("Targeting/Drive Angle to Face Target", driveAngleToFaceTarget);
    Logger.recordOutput("Targeting/Distance to Target", distanceToTarget);
    Logger.recordOutput("Targeting/Is Aimed", isAimed);
  }

  /**
   * Gets the field-relative heading that points the back of the robot at the target.
   *
   * @return the {@link Rotation2d} heading for the drivetrain to hold while shooting.
   */
  public Rotation2d getDriveAngleToFaceTarget() {
    return driveAngleToFaceTarget;
  }

  /**
   * Gets the distance from the shooter (back center of the robot) to the target.
   *
   * @return The distance to the target, in meters.
   */
  public double getDistanceToTarget() {
    return distanceToTarget;
  }

  /**
   * Whether the back of the robot is pointed at the target, within {@link
   * VisionConstants#AIM_TOLERANCE}.
   *
   * @return true if the robot is aimed at the target.
   */
  public boolean isAimed() {
    return isAimed;
  }

  /**
   * Whether the shooter (and hood) at the back of the robot is under one of the trenches.
   *
   * @return true if the hood must stay at its trench-safe angle.
   */
  public boolean isUnderTrench() {
    return isUnderTrench;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
