package team3647.frc2022.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import team3647.frc2022.constants.AutoConstants;
import team3647.frc2022.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;

    public AutoCommands(SwerveDrive drive, SwerveDriveKinematics driveKinematics) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
    }

    public PPSwerveControllerCommand getCommand(String pathName) {
        PathPlannerTrajectory trajectory = PathPlannerTrajectories.straightPath;
        switch (pathName) {
            case "straight":
                trajectory = PathPlannerTrajectories.straightPath;
                break;
            case "straight y":
                trajectory = PathPlannerTrajectories.straightYPath;
                break;
            case "eight":
                trajectory = PathPlannerTrajectories.eightPath;
                break;
            case "straight ninety":
                trajectory = PathPlannerTrajectories.straightNinetyPath;
                break;
            case "five ball":
                trajectory = PathPlannerTrajectories.fiveBallPath;
                break;
            default:
                trajectory = PathPlannerTrajectories.straightPath;
        }

        return new PPSwerveControllerCommand(
                trajectory,
                drive::getPose,
                driveKinematics,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                drive::setModuleStates,
                drive);
    }
}
