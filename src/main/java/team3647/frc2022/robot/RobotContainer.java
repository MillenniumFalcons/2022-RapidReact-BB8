package team3647.frc2022.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.LinkedList;
import java.util.List;
import team3647.frc2022.autonomous.AutoCommands;
import team3647.frc2022.autonomous.PathPlannerTrajectories;
import team3647.frc2022.commands.SwerveDriveNoAim;
import team3647.frc2022.constants.ColumnConstants;
import team3647.frc2022.constants.GlobalConstants;
import team3647.frc2022.constants.SwerveDriveConstants;
import team3647.frc2022.constants.TurretConstants;
import team3647.frc2022.constants.VisionConstants;
import team3647.frc2022.constants.WristIntakeConstants;
import team3647.frc2022.subsystems.Column;
import team3647.frc2022.subsystems.Superstructure;
import team3647.frc2022.subsystems.SwerveDrive;
import team3647.frc2022.subsystems.Turret;
import team3647.frc2022.subsystems.WristIntake;
import team3647.frc2022.subsystems.vision.VisionController;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.tracking.RobotTracker;
import team3647.lib.vision.Limelight;
import team3647.lib.vision.MultiTargetTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Joysticks mainController = new Joysticks(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(m_swerve, m_printer, m_visionController, m_wristIntake);

        // m_swerve.setDefaultCommand(
        //         new SwerveDriveTeleopBaseFalcon(
        //                 m_swerve,
        //                 mainController::getLeftStickX,
        //                 mainController::getLeftStickY,
        //                 mainController::getRightStickX,
        //                 mainController::getRightTriggerValue,
        //                 () -> {
        //                     var aimingParams = m_superstructure.getAimingParameters();
        //                     if (aimingParams == null) {
        //                         return new Pose2d();
        //                     }
        //                     return aimingParams.getFieldToGoal();
        //                 },
        //                 // () -> new Pose2d(),
        //                 () -> true));

        m_swerve.setDefaultCommand(
                new SwerveDriveNoAim(
                        m_swerve,
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        // () -> new Pose2d(),
                        () -> true));

        configureButtonBindings();
        configureSmartDashboardLogging();
        SmartDashboard.putNumber("Swerve Angle", 0.0);
        m_printer.addDouble("Intake Angle", () -> m_wristIntake.getDegrees());
        m_printer.addDouble("Raw Rotation", () -> m_swerve.getRawHeading());
        m_swerve.setOdometry(
                PathPlannerTrajectories.startStateStraightNinety,
                new Rotation2d(Units.degreesToRadians(0)));
        m_printer.addPose("Robot", m_swerve::getPose);
        m_printer.addPose(
                "Vision Pose",
                () -> {
                    var aimingParams = m_superstructure.getAimingParameters();
                    if (aimingParams == null) {
                        return new Pose2d();
                    }
                    return aimingParams.getFieldToGoal();
                });
    }

    private void configureButtonBindings() {
        // reset heading
        mainController.buttonA.whenPressed(() -> m_swerve.zeroHeading());
        // mainController.buttonY.whenPressed(new
        // TeleopAim(swerve, new Translation2d(5.0, 5.0)));
        // mainController.buttonB.whileActiveOnce(
        //         swerveCommands.variableVelocity(this::getSwerveAngle));
        mainController.rightTrigger.whileActiveOnce(
                m_superstructure.intakeCommands.deploy().perpetually());
        mainController.rightBumper.whileActiveOnce(
                m_superstructure.intakeCommands.retract().perpetually());
        mainController.leftBumper.whileActiveOnce(m_superstructure.feed());
        // mainController.buttonX.whenHeld(new InstantCommand(m_wristIntake::increaseDemand));
        // mainController.buttonB.whenHeld(new InstantCommand(m_wristIntake::increaseFF));
    }

    public void configureSmartDashboardLogging() {
        SmartDashboard.putNumber("Wrist Intake Pos", 0.0);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommands.getCommand("straight ninety");
    }

    public double getSwerveAngle() {
        return SmartDashboard.getNumber("Swerve Angle", 0.0);
    }

    public void updateTapeTranslations(List<Translation2d> translations) {
        List<Pose2d> poses = new LinkedList<>();
        translations.stream()
                .map((translation) -> new Pose2d(translation, new Rotation2d()))
                .map(m_flightDeck::getInFieldCoordinatesFromCamera)
                .forEach(poses::add);
        m_printer.getField().getObject("Tapes").setPoses(poses);
    }

    public final SwerveDrive m_swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kFrontLeftModule,
                    SwerveDriveConstants.kFrontRightModule,
                    SwerveDriveConstants.kBackLeftModule,
                    SwerveDriveConstants.kBackRightModule,
                    SwerveDriveConstants.kGyro);
    public final WristIntake m_wristIntake =
            new WristIntake(
                    WristIntakeConstants.kDeployMotor,
                    WristIntakeConstants.kIntakeMotor,
                    WristIntakeConstants.kDeployFeedForward,
                    WristIntakeConstants.kIntakeFeedForward,
                    WristIntakeConstants.motionConstraints,
                    GlobalConstants.kDt,
                    WristIntakeConstants.kIntakeNativeVelToSurfaceMpS,
                    WristIntakeConstants.kDeployNativeVelocityToDegpS,
                    WristIntakeConstants.kDeployNativePositionToDegrees,
                    WristIntakeConstants.kNominalVoltage,
                    WristIntakeConstants.maxDeployVelocityDegPerSec,
                    WristIntakeConstants.intakableDegree,
                    WristIntakeConstants.zeroDeg);
    public final Column m_column =
            new Column(
                    ColumnConstants.kColumnMotor,
                    ColumnConstants.kTopBanner,
                    ColumnConstants.kNativeVelToSurfaceMpS,
                    ColumnConstants.kPosConversion,
                    ColumnConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnConstants.kFeedForward);
    final Turret m_turret =
            new Turret(
                    TurretConstants.kTurretMotor,
                    TurretConstants.kFalconVelocityToDegpS,
                    TurretConstants.kFalconPositionToDegrees,
                    TurretConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    TurretConstants.kS,
                    TurretConstants.kMaxDegree,
                    TurretConstants.kMinDegree,
                    TurretConstants.kStartingAngle,
                    TurretConstants.kTurretProfile,
                    TurretConstants.kFeedForwards);

    final FlightDeck m_flightDeck =
            new FlightDeck(
                    new RobotTracker(
                            1.0,
                            VisionConstants.kRobotToTurretFixed,
                            m_swerve::getPose,
                            m_swerve::getTimestamp,
                            () -> new Rotation2d(),
                            m_swerve::getTimestamp),
                    new MultiTargetTracker(),
                    VisionConstants.kTurretToCamFixed);

    final VisionController m_visionController =
            new VisionController(
                    new Limelight("10.36.47.15", 0.018, VisionConstants.limelightConstants),
                    VisionConstants.kCenterGoalTargetConstants,
                    m_flightDeck::addVisionObservation,
                    this::updateTapeTranslations);

    final Superstructure m_superstructure =
            new Superstructure(m_flightDeck, m_wristIntake, m_column, m_turret);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter m_printer = GroupPrinter.getInstance();

    private final AutoCommands autoCommands =
            new AutoCommands(m_swerve, SwerveDriveConstants.kDriveKinematics);
}
