// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import team3647.frc2022.constants.AutoConstants;

/** Add your docs here. */
public class PathPlannerTrajectories {
    public static final PathPlannerTrajectory straightPath =
            PathPlanner.loadPath(
                    "straight",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateStraight = straightPath.getInitialPose();

    public static final PathPlannerTrajectory straightYPath =
            PathPlanner.loadPath(
                    "straight y",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateStraightY = straightYPath.getInitialPose();

    public static final PathPlannerTrajectory eightPath =
            PathPlanner.loadPath(
                    "eight",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateEight = eightPath.getInitialPose();

    public static final PathPlannerTrajectory straightNinetyPath =
            PathPlanner.loadPath(
                    "straight ninety",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateStraightNinety = straightNinetyPath.getInitialPose();

    public static final PathPlannerTrajectory fiveBallPath =
            PathPlanner.loadPath(
                    "five ball",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateFiveBall = fiveBallPath.getInitialPose();
}
