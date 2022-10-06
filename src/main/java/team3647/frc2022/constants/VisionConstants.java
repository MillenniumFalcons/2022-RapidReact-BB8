package team3647.frc2022.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import team3647.lib.vision.IVisionCamera.CamConstants;
import team3647.lib.vision.VisionTargetConstants;

public class VisionConstants {
    public static String kLimelightIP = "10.36.47.16";
    public static double kNetworkLatency = 0.06; // seconds
    public static double kCameraHeightMeters = Units.inchesToMeters(15.6);
    public static double kTopOfTapeHeightMeters = Units.inchesToMeters(60); // 104
    public static double kBottomOfTapeHeightMeters = Units.inchesToMeters(58.5); // 102

    public static double kInsideTopOfTapeHeightMeters = Units.inchesToMeters(58.5); // 102
    public static double kInsideBottomOfTapeHeightMeters = Units.inchesToMeters(57); // 100

    public static double kGoalDiameterMeters = Units.feetToMeters(5);
    public static Rotation2d kHorizontalToLens = Rotation2d.fromDegrees(0);
    public static double kVPH = 2.0 * Math.tan(Math.toRadians(49.7) / 2.0);
    public static double kVPW = 2.0 * Math.tan(Math.toRadians(59.8) / 2.0);
    public static CamConstants limelightConstants =
            new CamConstants(kCameraHeightMeters, kHorizontalToLens, kVPH, kVPW);
    public static VisionTargetConstants kCenterGoalTargetConstants =
            new VisionTargetConstants(
                    kTopOfTapeHeightMeters, kBottomOfTapeHeightMeters, 4, 2, kGoalDiameterMeters);

    public static Translation2d kRobotToTurretFixed = new Translation2d(0.0, 0.0);
    public static Pose2d kTurretToCamFixed = new Pose2d(0.35, -0.05, new Rotation2d());
}
