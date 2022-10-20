package team3647.frc2022.subsystems;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import team3647.frc2022.commands.ColumnCommands;
import team3647.frc2022.commands.IntakeCommands;
import team3647.frc2022.commands.turret.TurretCommands;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.vision.AimingParameters;

public class Superstructure {

    private AimingParameters aimingParameters;

    private double angleToTarget = 0;
    private double turretVelFF = 0.0;
    private final FlightDeck deck;
    private final WristIntake m_intake;
    private final Column m_column;
    private final Turret m_turret;

    public final IntakeCommands intakeCommands;
    public final ColumnCommands columnCommands;
    public final TurretCommands turretCommands;

    public Superstructure(FlightDeck deck, WristIntake m_intake, Column m_column, Turret m_turret) {
        this.deck = deck;
        this.m_intake = m_intake;
        this.m_column = m_column;
        this.m_turret = m_turret;
        intakeCommands = new IntakeCommands(m_intake);
        columnCommands = new ColumnCommands(m_column);
        turretCommands = new TurretCommands(m_turret);
    }

    public void periodic(double timestamp) {
        aimingParameters = deck.getLatestParameters();
        if (aimingParameters != null) {
            // flywheelVelocity =
            // FlywheelConstants.getFlywheelRPM(aimingParameters.getRangeMeters());
            // kickerVelocity = MathUtil.clamp(flywheelVelocity * 0.5, 0, 10);
            // hoodAngle = HoodContants.getHoodAngle1(aimingParameters.getRangeMeters());
            angleToTarget = aimingParameters.getTurretAngleToTarget().getDegrees();
            // turretSetpoint =
            //         m_turret.getAngle() + aimingParameters.getTurretAngleToTarget().getDegrees();
            Twist2d velocity = deck.getTracker().getMeasuredVelocity();
            double tangential_component =
                    aimingParameters.getRobotToTargetTransform().getRotation().getSin()
                            * velocity.dx
                            / aimingParameters.getRangeMeters();
            double angular_component = Units.radiansToDegrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local
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

    public double getAngleToTarget() {
        return this.angleToTarget;
    }

    public Command feed() {
        return columnCommands.getRunInwards();
    }

    public Command feederWithSensor(DoubleSupplier surfaceVelocity) {
        return columnCommands
                .getGoVariableVelocity(surfaceVelocity)
                .until(m_column::getTopBannerValue);
    }
}
