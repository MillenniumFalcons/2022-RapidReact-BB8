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
import team3647.frc2022.constants.FlywheelConstants;
import team3647.frc2022.constants.GlobalConstants;
import team3647.frc2022.constants.HoodContants;
import team3647.frc2022.constants.SwerveDriveConstants;
import team3647.frc2022.constants.TurretConstants;
import team3647.frc2022.constants.VisionConstants;
import team3647.frc2022.constants.WristIntakeConstants;
import team3647.frc2022.subsystems.Column;
import team3647.frc2022.subsystems.Flywheel;
import team3647.frc2022.subsystems.Hood;
import team3647.frc2022.subsystems.Intake;
import team3647.frc2022.subsystems.Superstructure;
import team3647.frc2022.subsystems.SwerveDrive;
import team3647.frc2022.subsystems.Turret;
import team3647.frc2022.subsystems.Wrist;
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
    private final Joysticks coController = new Joysticks(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(
                m_swerve,
                m_printer,
                m_visionController,
                m_column,
                m_turret,
                m_hood,
                m_wrist,
                m_intake,
                m_flywheel);

        configureButtonBindings();
        configureDefaultCommands();
        configureSmartDashboardLogging();

        m_swerve.setOdometry(
                PathPlannerTrajectories.startStateStraightNinety,
                new Rotation2d(Units.degreesToRadians(0)));
    }

    private void configureButtonBindings() {
        // main controller
        // reset heading
        mainController.buttonA.whenPressed(() -> m_swerve.zeroHeading());

        coController
                .buttonY
                .whileActiveOnce(m_superstructure.batterAccelerateAndShoot())
                .whileActiveOnce(m_superstructure.turretCommands.motionMagic(0).perpetually())
                .whileActiveOnce(
                        m_superstructure
                                .hoodCommands
                                .motionMagic(HoodContants.kBatterAngle)
                                .perpetually());

        // co controller
        coController.leftTrigger.whileActiveOnce(
                m_superstructure
                        .deployAndRunIntake(() -> 5)
                        .alongWith(m_superstructure.feederWithSensor(() -> 3)));

        coController.leftTrigger.whenInactive(m_superstructure.retractIntake());

        coController
                .rightTrigger
                .whileActiveOnce(m_superstructure.aimTurret())
                .whileActiveOnce(m_superstructure.fastAutoAccelerateAndShoot());
        // mainController
        //         .buttonB
        //         .whileActiveOnce(
        //                 new InstantCommand(
        //                         () -> m_flywheel.setSurfaceSpeed(this.getShooterDashboard()),
        //                         m_flywheel))
        //         .whileActiveOnce(
        //                 new InstantCommand(
        //                         () -> m_hood.setAngleMotionMagic(this.getHoodDashboard()),
        // m_hood))
        //         .whileActiveOnce(m_superstructure.columnCommands.getRunInwards());
    }

    private void configureDefaultCommands() {
        // aim with drivetrain
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
        m_hood.setDefaultCommand(
                m_superstructure.hoodCommands.autoAdjustAngle(m_superstructure::getAimedHoodAngle));
        m_flywheel.setDefaultCommand(
                m_superstructure.flywheelCommands.waitToSpinDownThenHold(
                        m_superstructure::getHoldVelocity));
        m_turret.setDefaultCommand(m_superstructure.turretCommands.holdPositionAtCall());
        m_column.setDefaultCommand(m_superstructure.feederManual(coController::getLeftStickY));
    }

    public void configureSmartDashboardLogging() {
        SmartDashboard.putNumber("Swerve Angle", 0.0);
        m_printer.addDouble("Raw Rotation", () -> m_swerve.getRawHeading());
        m_printer.addDouble("Turret go To", () -> m_superstructure.getAimedTurretSetpoint());
        m_printer.addDouble("Hood go To", () -> m_superstructure.getAimedHoodAngle());
        m_printer.addPose("Robot", m_swerve::getPose);
        m_printer.addDouble("Aimed Velkocity", () -> m_superstructure.getAimedFlywheelSurfaceVel());
        m_printer.addPose(
                "Vision Pose",
                () -> {
                    var aimingParams = m_superstructure.getAimingParameters();
                    if (aimingParams == null) {
                        return new Pose2d();
                    }
                    return aimingParams.getFieldToGoal();
                });
        SmartDashboard.putNumber("Hood Angle", 16.0);
        SmartDashboard.putNumber("Shooter Velocity", 5.0);
    }

    public double getShooterDashboard() {
        return SmartDashboard.getNumber("Shooter Velocity", 5.0);
    }

    public double getHoodDashboard() {
        return SmartDashboard.getNumber("Hood Angle", 16.0);
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
    final Hood m_hood =
            new Hood(
                    HoodContants.kHoodMotor,
                    HoodContants.kFalconVelocityToDegpS,
                    HoodContants.kFalconPositionToDegrees,
                    HoodContants.kNominalVoltage,
                    HoodContants.kS,
                    HoodContants.kCos,
                    GlobalConstants.kDt,
                    HoodContants.kMinDegree,
                    HoodContants.kMaxDegree,
                    HoodContants.kPosThersholdDeg);

    final Flywheel m_flywheel =
            new Flywheel(
                    FlywheelConstants.kMaster,
                    FlywheelConstants.kNativeVelToSurfaceMpS,
                    0,
                    FlywheelConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    FlywheelConstants.kFollower,
                    FlywheelConstants.kFeedForward);
    final Wrist m_wrist =
            new Wrist(
                    WristIntakeConstants.kDeployMotor,
                    WristIntakeConstants.kDeployNativeVelocityToDegpS,
                    WristIntakeConstants.kDeployNativePositionToDegrees,
                    WristIntakeConstants.kNominalVoltage,
                    WristIntakeConstants.kDeployS,
                    WristIntakeConstants.kDeployCos,
                    GlobalConstants.kDt,
                    WristIntakeConstants.kMinDegree,
                    WristIntakeConstants.kMaxDegree,
                    WristIntakeConstants.kPosThersholdDeg,
                    WristIntakeConstants.intakableDegree,
                    WristIntakeConstants.zeroDeg);

    final Intake m_intake =
            new Intake(
                    WristIntakeConstants.kIntakeMotor,
                    WristIntakeConstants.kIntakeNativeVelToSurfaceMpS,
                    0,
                    WristIntakeConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    WristIntakeConstants.kIntakeFeedForward);

    final FlightDeck m_flightDeck =
            new FlightDeck(
                    new RobotTracker(
                            1.0,
                            TurretConstants.kRobotToTurretFixed,
                            m_swerve::getPose,
                            m_swerve::getTimestamp,
                            m_turret::getRotation,
                            m_swerve::getTimestamp),
                    new MultiTargetTracker(),
                    TurretConstants.kTurretToCamFixed);

    final VisionController m_visionController =
            new VisionController(
                    new Limelight("10.36.47.15", 0.018, VisionConstants.limelightConstants),
                    VisionConstants.kCenterGoalTargetConstants,
                    m_flightDeck::addVisionObservation,
                    this::updateTapeTranslations);

    final Superstructure m_superstructure =
            new Superstructure(
                    m_flightDeck,
                    m_column,
                    m_turret,
                    m_hood,
                    m_flywheel,
                    m_wrist,
                    m_intake,
                    m_swerve::isStopped);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter m_printer = GroupPrinter.getInstance();

    private final AutoCommands autoCommands =
            new AutoCommands(m_swerve, SwerveDriveConstants.kDriveKinematics);
}
