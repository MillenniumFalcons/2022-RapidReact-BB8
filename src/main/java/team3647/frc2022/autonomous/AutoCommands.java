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

    public Command getStraight() {
        Command turretSequence =
                new WaitCommand(0.7)
                        .andThen(superstructure.turretCommands.motionMagic(0))
                        .andThen(new WaitCommand(0.2), superstructure.aimTurret());
        Command drivetrainSequence =
                CommandGroupBase.sequence(new WaitCommand(3), getPartCommand("straight"));
        Command intakeSequence =
                CommandGroupBase.sequence(
                        superstructure.retractIntake().withTimeout(0.6),
                        new WaitCommand(
                                2.4
                                        + PathPlannerTrajectories.straightPath.getTotalTimeSeconds()
                                        - 0.8),
                        superstructure.deployAndRunIntake(() -> 6).withTimeout(3),
                        superstructure
                                .retractIntake()
                                .alongWith(superstructure.intakeCommands.openLoopAndStop(0.3)));
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(0.6),
                        superstructure.fastAutoAccelerateAndShoot().withTimeout(2.4),
                        new WaitCommand(PathPlannerTrajectories.straightPath.getTotalTimeSeconds()),
                        superstructure.fastAutoAccelerateAndShoot());
        return CommandGroupBase.parallel(
                turretSequence, drivetrainSequence, shooterFeederSequence, intakeSequence);
    }

    public Command getSixBall() {
        Command turretSequence =
                new WaitCommand(0.7)
                        .andThen(superstructure.turretCommands.motionMagic(0))
                        .andThen(new WaitCommand(0.2), superstructure.aimTurret());
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(3.3),
                        getPartCommand("six ball bump 1"),
                        new WaitCommand(1.7),
                        getPartCommand("six ball bump 2"),
                        new WaitCommand(2),
                        getPartCommand("six ball bump 3"));
        Command intakeSequence =
                CommandGroupBase.sequence(
                        superstructure.retractIntake().withTimeout(0.6),
                        superstructure.intakeCommands.openLoopAndStop(0.4).withTimeout(2.5),
                        new WaitCommand(
                                PathPlannerTrajectories.sixBallBump1.getTotalTimeSeconds() - 0.8),
                        superstructure.deployAndRunIntake(() -> 6).withTimeout(1.7),
                        superstructure.retractIntake(),
                        new WaitCommand(
                                PathPlannerTrajectories.sixBallBump2.getTotalTimeSeconds() - 0.8),
                        superstructure.deployAndRunIntake(() -> 6).withTimeout(2.6),
                        superstructure.retractIntake(),
                        new WaitCommand(PathPlannerTrajectories.sixBallBump3.getTotalTimeSeconds()),
                        superstructure.intakeCommands.openLoopAndStop(0.4).withTimeout(1.7),
                        superstructure.deployAndRunIntake(() -> 6).withTimeout(1.5),
                        superstructure.retractIntake());
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(0.9),
                        superstructure.fastAutoAccelerateAndShoot().withTimeout(2.4),
                        new WaitCommand(
                                PathPlannerTrajectories.sixBallBump1.getTotalTimeSeconds() - 1),
                        superstructure.feederWithSensor(() -> 4).withTimeout(1.2),
                        superstructure.fastAutoAccelerateAndShoot().withTimeout(1.7),
                        new WaitCommand(
                                PathPlannerTrajectories.sixBallBump2.getTotalTimeSeconds() - 1),
                        superstructure.feederWithSensor(() -> 4).withTimeout(2.8),
                        new WaitCommand(PathPlannerTrajectories.sixBallBump3.getTotalTimeSeconds()),
                        superstructure.fastAutoAccelerateAndShoot());
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
