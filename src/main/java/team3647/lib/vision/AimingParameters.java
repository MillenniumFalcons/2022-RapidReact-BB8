package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** 254 AimingParameters class */
public class AimingParameters {
    public final int id;
    private final double rangeMeters;
    private final Pose2d fieldToTarget;
    private final Pose2d fieldToRobot;
    private final Transform2d robotToTarget;
    private final Transform2d turretToTarget;
    private final Rotation2d turretAngleToTarget;
    private final double lastSeenTimestamp;
    private final double stability;

    public AimingParameters(
            int id,
            Pose2d fieldToTurret,
            Pose2d fieldToRobot,
            Pose2d fieldToTarget,
            double lastSeenTimestamp,
            double stability) {
        this.id = id;
        this.fieldToTarget = fieldToTarget;
        this.turretToTarget = fieldToTarget.minus(fieldToTurret);
        this.fieldToRobot = fieldToRobot;
        this.robotToTarget = fieldToTarget.minus(fieldToRobot);
        this.rangeMeters = turretToTarget.getTranslation().getNorm();
        this.turretAngleToTarget =
                new Rotation2d(
                        turretToTarget.getTranslation().getX(),
                        turretToTarget.getTranslation().getY());
        this.lastSeenTimestamp = lastSeenTimestamp;
        this.stability = stability;
    }

    public double getStability() {
        return stability;
    }

    public double getLastSeenTimestamp() {
        return lastSeenTimestamp;
    }

    public Pose2d getFieldToGoal() {
        return fieldToTarget;
    }

    public Rotation2d getTurretAngleToTarget() {
        return turretAngleToTarget;
    }

    public Transform2d getRobotToTargetTransform() {
        return robotToTarget;
    }

    public Pose2d getFieldToRobot() {
        return fieldToRobot;
    }

    public double getRangeMeters() {
        return rangeMeters;
    }
}
