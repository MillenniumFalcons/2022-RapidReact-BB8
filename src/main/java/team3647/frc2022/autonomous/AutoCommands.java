package team3647.frc2022.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2022.constants.AutoConstants;
import team3647.frc2022.subsystems.Superstructure;
import team3647.frc2022.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;
    private final Superstructure superstructure;

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.driveKinematics = driveKinematics;
    }

    // intake and shooter feeder both require feeder
    public Command getStraightCommand() {
        Command turretSequence =
                new WaitCommand(0.5)
                        .andThen(superstructure.turretCommands.motionMagic(0))
                        .andThen(new WaitCommand(0.2), superstructure.aimTurret());
        Command drivetrainSequence =
                CommandGroupBase.sequence(new WaitCommand(1.8), getPartCommand("straight"));
        Command intakeSequence =
                CommandGroupBase.sequence(
                        superstructure.retractIntake().withTimeout(0.8),
                        new WaitCommand(
                                1.0
                                        + PathPlannerTrajectories.straightPath.getTotalTimeSeconds()
                                        - 1.2),
                        superstructure.retractIntake());
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(0.6),
                        superstructure.fastAutoAccelerateAndShoot().withTimeout(1.2),
                        new WaitCommand(
                                PathPlannerTrajectories.straightPath.getTotalTimeSeconds() + 2.0));
        return CommandGroupBase.parallel(
                turretSequence, drivetrainSequence, shooterFeederSequence, intakeSequence);
    }

    public PPSwerveControllerCommand getPartCommand(String pathName) {
        PathPlannerTrajectory trajectory = PathPlannerTrajectories.straightPath;
        switch (pathName) {
            case "straight":
                trajectory = PathPlannerTrajectories.straightPath;
                break;
            case "six ball bump 1":
                trajectory = PathPlannerTrajectories.sixBallBump1;
                break;
            case "six ball bump 2":
                trajectory = PathPlannerTrajectories.sixBallBump2;
                break;
            case "six ball bump 3":
                trajectory = PathPlannerTrajectories.sixBallBump3;
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
