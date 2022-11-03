package team3647.frc2022.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.vision.CircleFitter;
import team3647.lib.vision.IVisionCamera;
import team3647.lib.vision.IVisionCamera.CamConstants;
import team3647.lib.vision.IVisionCamera.VisionInputs;
import team3647.lib.vision.IVisionCamera.VisionPipeline;
import team3647.lib.vision.IVisionCamera.VisionPoint;
import team3647.lib.vision.VisionTargetConstants;

public class VisionController implements PeriodicSubsystem {
    private final IVisionCamera camera;
    private final VisionTargetConstants targetConstants;
    private final BiConsumer<Double, Translation2d> translationConsumer;
    private final PeriodicIO periodicIO;
    private static final Rotation2d PiOver2 = new Rotation2d(Math.PI / 2.0);
    private static final double kCircleFitPrecision = 0.01;
    private static final double kNetworklatency = 0.06;
    private final double targetAverageHeightMeters;
    private final Consumer<List<Translation2d>> tapeTranslationConsumer;
    private static Translation2d targeTranslation = new Translation2d();

    private static final class PeriodicIO {
        public final VisionInputs inputs = new VisionInputs();
        public double lastTimestamp = 0.0;
    }

    public VisionController(
            IVisionCamera camera,
            VisionTargetConstants targetConstants,
            BiConsumer<Double, Translation2d> translationConsumer,
            Consumer<List<Translation2d>> tapeTranslation) {
        this.camera = camera;
        this.targetConstants = targetConstants;
        this.translationConsumer = translationConsumer;
        this.periodicIO = new PeriodicIO();
        targetAverageHeightMeters =
                (targetConstants.kTopTargetHeightMeters + targetConstants.kBottomTargetHeightMeters)
                        / 2.0;
        tapeTranslationConsumer = tapeTranslation;
    }

    @Override
    public void readPeriodicInputs() {
        camera.writeToInputs(periodicIO.inputs);
        if (periodicIO.lastTimestamp == periodicIO.inputs.captureTimestamp) {
            return;
        }

        periodicIO.lastTimestamp = periodicIO.inputs.captureTimestamp;
        SmartDashboard.putNumber("SKEW", periodicIO.inputs.skew);
        SmartDashboard.putNumber("TX", periodicIO.inputs.angleToVisionCenter);
        // Rejects if the target group is too skewed (we noticed this messes up the aiming)
        // if (periodicIO.inputs.skew > 8 && periodicIO.inputs.skew < 75) {
        //     return;
        // }

        int targetCount = periodicIO.inputs.corners.size() / targetConstants.kPointsPerTarget;
        if (targetCount < targetConstants.kMinTargetCount) {
            return;
        }
        List<Translation2d> camToTargetTranslations = new LinkedList<>();
        // accessing the arrays as if they are 2d linear (c 2d arrays)

        periodicIO.inputs.corners.stream()
                .map(this::getTranslationDefault)
                .filter(Objects::nonNull)
                .forEach(camToTargetTranslations::add);
        if (camToTargetTranslations.size()
                < targetConstants.kMinTargetCount * targetConstants.kPointsPerTarget) {
            return;
        }
        tapeTranslationConsumer.accept(camToTargetTranslations);
        Translation2d closest = camToTargetTranslations.get(0);
        for (var translation : camToTargetTranslations) {
            if (translation.getNorm() < closest.getNorm()) {
                closest = translation;
            }
        }

        SmartDashboard.putNumber("Distance to closest", closest.getNorm());
        double a1a2 =
                camera.getConstants().kHorizontalToLens.getRadians()
                        + Math.toRadians(periodicIO.inputs.pitchToVisionCenter);
        double naiveDistance =
                (targetConstants.kBottomTargetHeightMeters
                                - this.camera.getConstants().kCameraHeightMeters)
                        / Math.tan(a1a2);
        SmartDashboard.putNumber("Naive Distance", naiveDistance);

        var fitCircle =
                CircleFitter.fit(
                        targetConstants.kTargetDiameterMeters / 2.0,
                        camToTargetTranslations,
                        kCircleFitPrecision);
        var angleToCenterCircle = new Rotation2d(fitCircle.getX(), fitCircle.getY());

        // The difference between angle to center of circle, and angle to center of vision tape
        // group should be at most 10*
        if (Math.abs(angleToCenterCircle.getDegrees() - periodicIO.inputs.angleToVisionCenter)
                > 10) {
            return;
        }
        SmartDashboard.putNumber("Degrees to circle", angleToCenterCircle.getDegrees());
        SmartDashboard.putNumber("Meters to circle", fitCircle.getNorm());
        synchronized (translationConsumer) {
            translationConsumer.accept(
                    periodicIO.inputs.captureTimestamp
                            - SmartDashboard.getNumber("extra latency", 0.0),
                    fitCircle);
        }
    }

    private Translation2d getTranslationDefault(VisionPoint corner) {
        return solveTranslationToTarget(
                corner,
                this.targetAverageHeightMeters,
                this.camera.getConstants(),
                this.camera.getPipeline());
    }

    /**
     * ONLY for 4 corners
     *
     * @param corners
     * @param average
     */
    static List<VisionPoint> sortCorners(List<VisionPoint> corners, VisionPoint average) {
        // if (corners.size() != 4) {
        //     throw new UnsupportedOperation("Corners needs exactly 4 elements");
        // }
        double minAngleInPosDirection = Math.PI;
        double maxAngleInNegDirection = -Math.PI;
        int topLeftIdx = -1;
        int topRightIdx = -1; // garbage values
        int bottomIdx1 = -1;
        int bottomIdx2 = -1;
        for (int i = 0; i < corners.size(); i++) {
            VisionPoint corner = corners.get(i);
            // flip y because 0,0 is the top in the picture
            double angleFromCenterToCorner =
                    new Rotation2d(corner.x - average.x, average.y - corner.y)
                            .minus(PiOver2)
                            .getRadians();
            if (angleFromCenterToCorner > 0
                    && angleFromCenterToCorner
                            < minAngleInPosDirection) { // One of the left corners
                minAngleInPosDirection = angleFromCenterToCorner;
                bottomIdx1 = topLeftIdx;
                topLeftIdx = i;
            } else if (angleFromCenterToCorner > maxAngleInNegDirection) {
                maxAngleInNegDirection = angleFromCenterToCorner;
                bottomIdx2 = topRightIdx;
                topRightIdx = i;
            }
        }
        List<VisionPoint> newCorners = new LinkedList<>();
        if (topLeftIdx != -1) {
            newCorners.add(corners.get(topLeftIdx));
        }
        if (topRightIdx != -1) {
            newCorners.add(corners.get(topRightIdx));
        }
        if (bottomIdx1 != -1) {
            newCorners.add(corners.get(bottomIdx1));
        }
        if (bottomIdx2 != -1) {
            newCorners.add(corners.get(bottomIdx2));
        }
        return newCorners;
    }

    public static Translation2d solveTranslationToTarget(
            VisionPoint corner,
            double targetHeightMeters,
            CamConstants camConstants,
            VisionPipeline camPipeline) {
        double yPixels = corner.x;
        double zPixels = corner.y;
        // robot frame, y is left right, z is up down, x is front back
        // Limlieght additional theory
        double halfWidth = camPipeline.width / 2.0;
        double halfHeight = camPipeline.height / 2.0;
        double nY = -(yPixels - halfWidth) / halfWidth;
        double nZ = -(zPixels - halfHeight) / halfHeight;
        Translation2d xzPlaneTranslation =
                new Translation2d(1.0, camConstants.kVPH / 2.0 * nZ)
                        .rotateBy(camConstants.kHorizontalToLens);
        double x = xzPlaneTranslation.getX();
        double y = camConstants.kVPW / 2.0 * nY;
        // This plane is the XZ plane, so the y component of the translation is actually Z
        double z = xzPlaneTranslation.getY();

        double heightDiff = camConstants.kCameraHeightMeters - targetHeightMeters;
        if ((z < 0.0) == (heightDiff > 0.0)) {
            double scale = heightDiff / -z;
            double range = Math.hypot(x, y) * scale;
            Rotation2d angleToTarget = new Rotation2d(x, y);
            return new Translation2d(
                    range * angleToTarget.getCos(), range * angleToTarget.getSin());
        }
        return null;
        // return new Translation2d(range * angleToTarget.getCos(), range * angleToTarget.getSin());
    }

    @Override
    public String getName() {
        return "VisionController";
    }
}
