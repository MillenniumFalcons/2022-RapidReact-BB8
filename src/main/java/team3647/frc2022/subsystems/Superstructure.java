package team3647.frc2022.subsystems;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.vision.AimingParameters;

public class Superstructure {

    private AimingParameters aimingParameters;

    private double angleToTarget = 0;
    private double turretVelFF = 0.0;
    private final FlightDeck deck;

    public Superstructure(FlightDeck deck) {
        this.deck = deck;
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
}
