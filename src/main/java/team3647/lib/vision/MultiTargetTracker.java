package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedList;
import java.util.List;

public class MultiTargetTracker {
    private final List<TrackedTarget> trackedTargets = new LinkedList<>();
    private int nextId = 0;

    public void update(double timestamp, Iterable<Pose2d> fieldToGoals) {
        for (Pose2d target : fieldToGoals) {

            boolean foundCorrespondingTarget = false;
            for (TrackedTarget tracked : trackedTargets) {
                if (foundCorrespondingTarget) {
                    tracked.removeOldObservations();
                } else {
                    foundCorrespondingTarget = tracked.attemptUpdate(timestamp, target);
                    SmartDashboard.putBoolean("Found Target To update", foundCorrespondingTarget);
                }
            }
            if (!foundCorrespondingTarget) {
                System.out.println("Found new target");
                trackedTargets.add(new TrackedTarget(timestamp, nextId++, target));
            }
        }
        removeDeadTargets();
    }

    private void removeDeadTargets() {
        trackedTargets.removeIf(tracked -> !tracked.isAlive());
    }

    public boolean hasTracks() {
        return !trackedTargets.isEmpty();
    }

    public List<TrackedTarget> getTrackedTargets() {
        return trackedTargets;
    }
}
