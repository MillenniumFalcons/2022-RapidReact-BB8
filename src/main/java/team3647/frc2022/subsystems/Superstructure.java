package team3647.frc2022.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2022.commands.ColumnCommands;
import team3647.frc2022.commands.FlywheelCommands;
import team3647.frc2022.commands.HoodCommands;
import team3647.frc2022.commands.IntakeCommands;
import team3647.frc2022.commands.WristCommands;
import team3647.frc2022.commands.turret.TurretCommands;
import team3647.frc2022.constants.ColumnConstants;
import team3647.frc2022.constants.FlywheelConstants;
import team3647.frc2022.constants.HoodContants;
import team3647.frc2022.constants.TurretConstants;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.vision.AimingParameters;

public class Superstructure {

    private AimingParameters aimingParameters;
    private double flywheelVelocity = 0;
    private double angleToTarget = 0;
    private double kickerVelocity = 0;
    private double hoodAngle = 16;
    private double turretVelFF = 0.0;
    private double turretSetpoint = TurretConstants.kStartingAngle;

    private final FlightDeck deck;
    private final Column m_column;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Flywheel m_flywheel;
    private final Wrist m_wrist;
    private final Intake m_intake;

    public final ColumnCommands columnCommands;
    public final TurretCommands turretCommands;
    public final HoodCommands hoodCommands;
    public final FlywheelCommands flywheelCommands;
    public final WristCommands wristCommands;
    public final IntakeCommands intakeCommands;

    private final BooleanSupplier drivetrainStopped;

    public Superstructure(
            FlightDeck deck,
            Column m_column,
            Turret m_turret,
            Hood m_hood,
            Flywheel m_flywheel,
            Wrist m_wrist,
            Intake m_intake,
            BooleanSupplier drivetrainStopped) {
        this.deck = deck;
        this.m_column = m_column;
        this.m_turret = m_turret;
        this.m_flywheel = m_flywheel;
        this.m_wrist = m_wrist;
        this.m_intake = m_intake;
        this.m_hood = m_hood;
        this.drivetrainStopped = drivetrainStopped;
        columnCommands = new ColumnCommands(m_column);
        turretCommands = new TurretCommands(m_turret);
        hoodCommands = new HoodCommands(m_hood);
        flywheelCommands = new FlywheelCommands(m_flywheel);
        wristCommands = new WristCommands(m_wrist);
        intakeCommands = new IntakeCommands(m_intake);
    }

    public void periodic(double timestamp) {
        aimingParameters = deck.getLatestParameters();
        if (aimingParameters != null) {
            flywheelVelocity = FlywheelConstants.getFlywheelRPM(aimingParameters.getRangeMeters());
            kickerVelocity = MathUtil.clamp(flywheelVelocity * 0.5, 0, 10);
            hoodAngle = HoodContants.getHoodAngle1(aimingParameters.getRangeMeters());
            angleToTarget = aimingParameters.getTurretAngleToTarget().getDegrees();
            turretSetpoint =
                    m_turret.getAngle() + aimingParameters.getTurretAngleToTarget().getDegrees();
            Twist2d velocity = deck.getTracker().getMeasuredVelocity();
            double tangential_component =
                    aimingParameters.getRobotToTargetTransform().getRotation().getCos()
                            * velocity.dy
                            / aimingParameters.getRangeMeters();
            double angular_component = Units.radiansToDegrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
            turretVelFF = -(angular_component + tangential_component);
        }
    }

    public double getDistanceToTarget() {
        if (aimingParameters == null) {
            return 0;
        }
        return aimingParameters.getRangeMeters();
    }

    public AimingParameters getAimingParameters() {
        return this.aimingParameters;
    }

    public Command aimTurret() {
        return turretCommands.aim(this::getAimedTurretSetpoint, this::getAimedTurretVelocity);
    }

    public double getAimedTurretVelocity() {
        return this.turretVelFF;
    }

    public double getAngleToTarget() {
        return this.angleToTarget;
    }

    public Command feederWithSensor(DoubleSupplier surfaceVelocity) {
        return columnCommands
                .getGoVariableVelocity(surfaceVelocity)
                .until(m_column::getTopBannerValue);
    }

    public Command deployAndRunIntake(DoubleSupplier surfaceVelocity) {
        return wristCommands.deploy().andThen(intakeCommands.runClosedLoop(surfaceVelocity));
        // return intakeCommands.runClosedLoop(surfaceVelocity);
    }

    public Command retractIntake() {
        return wristCommands.retract();
    }

    // for testing only
    public Command deployIntake() {
        return wristCommands.deploy();
    }

    // for testing only
    public Command setHood() {
        return hoodCommands.motionMagic(35);
    }

    public boolean getFlywheelReady(DoubleSupplier expectedVelocity, double threshold) {
        return Math.abs(m_flywheel.getVelocity() - expectedVelocity.getAsDouble()) < threshold;
    }

    public boolean ballWentThrough(
            DoubleSupplier flywheel, DoubleSupplier kicker, double threshold) {
        return m_flywheel.getVelocity() + threshold < flywheel.getAsDouble();
    }

    public boolean readyToBatter() {
        return getFlywheelReady(this::getBatterVelocity, 2)
                && Math.abs(m_hood.getAngle() - HoodContants.kBatterAngle) < 1
                && Math.abs(m_flywheel.getVelocity()) > 5;
    }

    public boolean batterBallWentThrough() {
        return ballWentThrough(this::getBatterVelocity, () -> ColumnConstants.kShootVelocity, 1);
    }

    public Command batterAccelerateAndShoot() {
        return new WaitUntilCommand(() -> Math.abs(m_turret.getAngle()) < 3)
                .andThen(
                        fastAccelerateAndShoot(
                                this::getBatterVelocity,
                                () -> ColumnConstants.kShootVelocity,
                                this::readyToBatter,
                                0));
    }

    public Command fastAutoAccelerateAndShoot() {
        return fastAutoAccelerateAndShoot(5, 0, 0);
    }

    public Command fastAutoAccelerateAndShoot(
            double feederSpeed, double delayBetweenShots, double timeoutAfterDrivetrainStops) {
        return fastAccelerateAndShoot(
                this::getAimedFlywheelSurfaceVel,
                this::getAimedKickerVelocity,
                this::readyToAutoShoot,
                0);
    }

    public Command fastAccelerateAndShoot(
            DoubleSupplier flywheelVelocity,
            DoubleSupplier kickerVelocity,
            BooleanSupplier readyToShoot,
            double delayAfterDrivetrainStops) {

        return CommandGroupBase.parallel(
                flywheelCommands.variableVelocity(flywheelVelocity),
                CommandGroupBase.sequence(
                        new ConditionalCommand(
                                new InstantCommand(),
                                new WaitUntilCommand(drivetrainStopped)
                                        .andThen(new WaitCommand(delayAfterDrivetrainStops)),
                                drivetrainStopped),
                        new WaitUntilCommand(readyToShoot),
                        columnCommands
                                .getGoVariableVelocity(kickerVelocity)
                                .alongWith(intakeCommands.openLoopAndStop(0.3))));
    }

    public double getAimedFlywheelSurfaceVel() {
        return flywheelVelocity;
    }

    public double getAimedKickerVelocity() {
        return kickerVelocity;
    }

    public boolean readyToAutoShoot() {
        double turretSetpointNormalized =
                getAimedTurretSetpoint() - 360.0 * Math.round(getAimedTurretSetpoint() / 360.0);
        return Math.abs(m_flywheel.getVelocity() - getAimedFlywheelSurfaceVel()) < 0.2
                && Math.abs(m_hood.getAngle() - getAimedHoodAngle()) < 1
                && Math.abs(m_flywheel.getVelocity()) > 5
                && Math.abs(getAngleToTarget()) < 1;
    }

    public double getBatterVelocity() {
        return FlywheelConstants.kBatterVelocity;
    }

    public double getAimedHoodAngle() {
        return hoodAngle;
    }

    public double getAimedTurretSetpoint() {
        return this.turretSetpoint;
    }

    public double getHoldVelocity() {
        return 4.0;
    }

    public Command feederManual(DoubleSupplier manual) {
        return new RunCommand(
                () -> {
                    var leftY = manual.getAsDouble();
                    m_column.setOpenloop(leftY * leftY * leftY * 0.5);
                },
                m_column);
    }

    // public Command rejectBallTop() {
    //     return CommandGroupBase.sequence(
    //                     turretCommands.motionMagic(-30, 5),
    //                     CommandGroupBase.parallel(
    //                             flywheelCommands.openloop(0.3),
    //                             feederWithSensor(() -> 3).withTimeout(0.5)),
    //                     CommandGroupBase.parallel(
    //                                     flywheelCommands.openloop(0.3),
    //                                     columnCommands.getRunInwards())
    //                             .withTimeout(0.7))
    //             .withTimeout(2);
    // }
}
