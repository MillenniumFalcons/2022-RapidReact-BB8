package team3647.lib.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import team3647.lib.vision.AimingParameters;
import team3647.lib.vision.MultiTargetTracker;
import team3647.lib.vision.TrackedTarget;
import team3647.lib.vision.TrackedTarget.TrackedTargetComparator;

public class FlightDeck {
    private final RobotTracker robotTracker;
    private final MultiTargetTracker targetTracker;
    public static double maxAge;
    private final Transform2d kTurretToCamTransform;
    private int lastTargetId = 0;

    public FlightDeck(
            RobotTracker robotTracker, MultiTargetTracker targetTracker, Pose2d kTurretToCamFixed) {
        this.robotTracker = robotTracker;
        this.targetTracker = targetTracker;
        kTurretToCamTransform =
                new Transform2d(
                        kTurretToCamFixed.getTranslation(), kTurretToCamFixed.getRotation());
    }

    public synchronized void addVisionObservation(double timestamp, Translation2d camToGoal) {
        Pose2d fieldToTurret = robotTracker.getFieldToTurret(timestamp);
        if (fieldToTurret == null || camToGoal == null) {
            return;
        }
        Transform2d camToGoalTransform = new Transform2d(camToGoal, new Rotation2d());
        Pose2d fieldToTarget =
                fieldToTurret.transformBy(kTurretToCamTransform).transformBy(camToGoalTransform);
        targetTracker.update(
                timestamp,
                List.of(new Pose2d(fieldToTarget.getTranslation(), Rotation2d.fromDegrees(180))));
        // targetTracker.update(timestamp, List.of(new Pose2d(5.5, 5.5, new Rotation2d())));
    }

    public synchronized Pose2d getInFieldCoordinatesFromCamera(Pose2d pose) {
        Pose2d fieldToTurret = robotTracker.getFieldToTurret(Timer.getFPGATimestamp());
        if (fieldToTurret == null || pose == null) {
            return new Pose2d();
        }
        Transform2d camToGoalTransform = new Transform2d(new Pose2d(), pose);
        return fieldToTurret.transformBy(kTurretToCamTransform).transformBy(camToGoalTransform);
    }

    public Pose2d getFieldToCamera(Transform2d turretToCam) {
        var ftt = robotTracker.getFieldToTurret(Timer.getFPGATimestamp());
        if (ftt == null) {
            return null;
        }
        return ftt.transformBy(turretToCam);
    }

    private synchronized AimingParameters getAimingParameters(int lastTargetId) {
        List<TrackedTarget> targets = targetTracker.getTrackedTargets();
        SmartDashboard.putNumber("Number of targets", targets.size());
        if (targets.isEmpty()) {
            return null;
        }
        double timestamp = Timer.getFPGATimestamp();
        TrackedTargetComparator comparator =
                new TrackedTargetComparator(0.0, 10.0, timestamp, 100.0, lastTargetId);
        targets.sort(comparator);
        TrackedTarget bestTarget = targets.get(0);
        for (TrackedTarget target : targets) {
            if (target.getLatestTimestamp() > timestamp - maxAge) {
                bestTarget = target;
            }
        }
        return new AimingParameters(
                bestTarget.id,
                robotTracker.getFieldToTurret(timestamp),
                robotTracker.getFieldToRobot(timestamp),
                bestTarget.getSmoothedPosition(),
                bestTarget.getLatestTimestamp(),
                bestTarget.getStability());
    }

    public AimingParameters getLatestParameters() {
        return getAimingParameters(lastTargetId);
    }

    public RobotTracker getTracker() {
        return robotTracker;
    }
}
