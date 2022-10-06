package team3647.lib.vision;

public class VisionTargetConstants {
    public final double kTopTargetHeightMeters;
    public final double kBottomTargetHeightMeters;
    public final int kPointsPerTarget;
    public final int kMinTargetCount;
    public final double kTargetDiameterMeters;

    public VisionTargetConstants(
            double topTargetHeightMeters,
            double bottomTargetHeightMeters,
            int pointsPerTarget,
            int minTargetCount,
            double targetDiameterMeters) {
        this.kTopTargetHeightMeters = topTargetHeightMeters;
        this.kBottomTargetHeightMeters = bottomTargetHeightMeters;
        this.kPointsPerTarget = pointsPerTarget;
        this.kMinTargetCount = minTargetCount;
        this.kTargetDiameterMeters = targetDiameterMeters;
    }
}
