package team3647.frc2022.subsystems.vision;

import java.util.Objects;
import team3647.frc2022.subsystems.vision.Limelight.Data;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.utils.RollingAverage;
import team3647.lib.vision.IVisionCamera;
import team3647.lib.vision.VisionTargetConstants;
import team3647.lib.wpi.HALMethods;

public class VisionController implements PeriodicSubsystem {

    public static class CamConstants {
        public final double kGoalHeight;
        public final double kCameraHeight;
        public final double kCameraAngle;
        public final double kImageCaptureLatency;

        public CamConstants(
                double goalHeight,
                double cameraHeight,
                double cameraAngle,
                double imageCaptureLatency) {
            this.kGoalHeight = goalHeight;
            this.kCameraHeight = cameraHeight;
            this.kCameraAngle = cameraAngle;
            kImageCaptureLatency = imageCaptureLatency;
        }
    }

    public static class PeriodicIO {
        public boolean validTarget;
        public double x;
        public double y;
        public double area;
        public double skew;
        public double latency;
        public double range;

        public Limelight.LEDMode ledMode = Limelight.LEDMode.DEFAULT;
        public Limelight.CamMode camMode = Limelight.CamMode.VISION;
        public Limelight.StreamMode streamMode = Limelight.StreamMode.Limelight;
        public Pipeline pipeline = Pipeline.CLOSE;
    }

    private final Limelight limelight;
    private final IVisionCamera.CamConstants m_constants;
    private final VisionTargetConstants m_target;
    private boolean outputsHaveChanged = false;
    private PeriodicIO periodicIO = new PeriodicIO();

    private RollingAverage xAverage = new RollingAverage();
    private RollingAverage yAverage = new RollingAverage();
    private RollingAverage areaAverage = new RollingAverage();
    private RollingAverage skewAverage = new RollingAverage();
    private RollingAverage rangeAverage = new RollingAverage();

    public enum Pipeline {
        FAR(0),
        CLOSE(1),
        OUTSIDE_CLOUDY(2),
        SUNNY_TARGETING(3);

        public final int id;

        Pipeline(int id) {
            this.id = id;
        }
    }

    public VisionController(
            String camIP, IVisionCamera.CamConstants constants, VisionTargetConstants target) {
        Objects.requireNonNull(camIP);
        Objects.requireNonNull(constants);
        limelight = new Limelight(camIP);
        m_constants = constants;
        m_target = target;
        setPipeline(Pipeline.CLOSE);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.validTarget = (int) limelight.get(Data.VALID_TARGET) == 1;

        if (periodicIO.validTarget) {
            periodicIO.x = limelight.get(Data.X);
            xAverage.add(periodicIO.x);

            periodicIO.y = limelight.get(Data.Y);
            yAverage.add(periodicIO.y);

            periodicIO.area = limelight.get(Data.AREA);
            areaAverage.add(periodicIO.area);

            periodicIO.skew = limelight.get(Data.SKEW);
            skewAverage.add(periodicIO.skew);
        }
        periodicIO.latency = limelight.get(Data.LATNECY);

        periodicIO.range = calculateRange(getFilteredPitch());
        rangeAverage.add(periodicIO.range);
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsHaveChanged) {
            limelight.camMode(periodicIO.camMode);
            limelight.ledMode(periodicIO.ledMode);
            limelight.streamMode(periodicIO.streamMode);
            limelight.setPipeline(periodicIO.pipeline.id);
            HALMethods.sendDSWarning("limelight changed modes!");
            outputsHaveChanged = false;
        }
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public double getDistance() {
        return periodicIO.range;
    }

    public double getFilteredDistance() {
        return rangeAverage.getAverage();
    }

    public double getYaw() {
        return periodicIO.x;
    }

    public double getFilteredYaw() {
        return xAverage.getAverage();
    }

    public boolean isValid() {
        return periodicIO.validTarget;
    }

    public double getPitch() {
        return periodicIO.y;
    }

    public double getFilteredPitch() {
        return yAverage.getAverage();
    }

    private double calculateRange(double yDegrees) {
        return (m_target.kTopTargetHeightMeters - m_constants.kCameraHeightMeters)
                / Math.tan(Math.toRadians(yDegrees + m_constants.kHorizontalToLens.getDegrees()));
    }

    public void setCamMode(Limelight.CamMode mode) {
        periodicIO.camMode = mode;
        outputsHaveChanged = true;
    }

    public void setStreamMode(Limelight.StreamMode mode) {
        periodicIO.streamMode = mode;
        outputsHaveChanged = true;
    }

    public void setLedMode(Limelight.LEDMode mode) {
        periodicIO.ledMode = mode;
        outputsHaveChanged = true;
    }

    public void setPipeline(Pipeline pipeline) {
        periodicIO.pipeline = pipeline;
        outputsHaveChanged = true;
    }

    @Override
    public String getName() {
        return "VisionController";
    }
}
