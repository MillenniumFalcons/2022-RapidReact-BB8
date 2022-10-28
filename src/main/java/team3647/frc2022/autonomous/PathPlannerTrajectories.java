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
                    "3 ball straight",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateStraight = straightPath.getInitialPose();

    public static final PathPlannerTrajectory sixBallBump1 =
            PathPlanner.loadPath(
                    "6 ball bump 1",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateSixBallBump1 = sixBallBump1.getInitialPose();

    public static final PathPlannerTrajectory sixBallBump2 =
            PathPlanner.loadPath(
                    "6 ball bump 2",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateSixBallBump2 = sixBallBump2.getInitialPose();

    public static final PathPlannerTrajectory sixBallBump3 =
            PathPlanner.loadPath(
                    "6 ball bump 2",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d startStateSixBallBump3 = sixBallBump3.getInitialPose();
}
