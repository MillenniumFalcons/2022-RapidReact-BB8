package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;
import team3647.lib.team254.util.InterpolatingDouble;
import team3647.lib.team254.util.InterpolatingTreeMap;

public final class FlywheelConstants {

    public static final TalonFXInvertType kMasterInverted = TalonFXInvertType.CounterClockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final boolean kCurrentLimitingEnable = true;
    public static final double kStallCurrent = 10;
    public static final double kMaxCurrent = 100;
    public static final double kMaxCurrentDurationSec = 1;

    public static final double kS = 0.261326; // 0.104902; // 0.57; // 0.74;
    public static final double kV = 0.547855; // 0.2;
    public static final double kA = 0.0238; // 0;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static final double kNominalVoltage = 10;

    public static final TalonFX kMaster = new LazyTalonFX(GlobalConstants.FlywheelIds.kMasterId);
    public static final TalonFX kFollower =
            new LazyTalonFX(GlobalConstants.FlywheelIds.kFollowerId);
    public static final double kGearboxReduction = 24 / 36.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            10 * kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    public static final double kLowGoalVelocity = 3;
    public static final double kBatterVelocity = 15;
    private static final double kOffset = 1.0;
    private static final double kFarOffset = -0.4;

    public static final double[][] kFlywheelMap2 = {
        {Units.feetToMeters(2) + GlobalConstants.kCenterOffsetMeters, 14},
        {Units.feetToMeters(4) + GlobalConstants.kCenterOffsetMeters, 13.5 + kOffset},
        {Units.feetToMeters(6) + GlobalConstants.kCenterOffsetMeters, 13 + kOffset},
        {Units.feetToMeters(8) + GlobalConstants.kCenterOffsetMeters, 13 + kOffset},
        {Units.feetToMeters(10) + GlobalConstants.kCenterOffsetMeters, 13.5 + kOffset},
        {Units.feetToMeters(12) + GlobalConstants.kCenterOffsetMeters, 15 + kOffset},
        {Units.feetToMeters(14) + GlobalConstants.kCenterOffsetMeters, 15.5 + kOffset},
        {Units.feetToMeters(16) + GlobalConstants.kCenterOffsetMeters, 16.8 + kOffset},
    };

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelAutoAimMap = new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelAutoAimMap2 = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelVelVoltage = new InterpolatingTreeMap<>();
    public static double constantVelocityMpS = 5;

    static {
        kMasterConfig.slot0.kP = 0.03; // 0.05;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0; // 0.08;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMasterConfig.supplyCurrLimit.enable = kCurrentLimitingEnable;
        kMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = kMaxCurrentDurationSec;
        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.enableVoltageCompensation(true);
        kFollower.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kFollower.enableVoltageCompensation(true);
        kMaster.setInverted(kMasterInverted);
        kMaster.enableVoltageCompensation(true);
        kFollower.enableVoltageCompensation(true);

        for (double[] pair : kFlywheelMap2) {
            kFlywheelAutoAimMap.put(
                    new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public static double getFlywheelRPM(double range) {
        InterpolatingDouble d = kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range));

        return d == null ? 10.5 : MathUtil.clamp(d.value, 7, 35);
    }

    public static double getFlywheelVoltage(double velocity) {
        InterpolatingDouble d =
                kFlywheelVelVoltage.getInterpolated(new InterpolatingDouble(velocity));
        // if (velocity < kFlywheelVoltageMap[0][0]
        //         || velocity > kFlywheelVoltageMap[kFlywheelVoltageMap.length - 1][0]) {
        //     return kFeedForward.calculate(velocity);
        // }
        return d == null ? kFeedForward.calculate(velocity) : MathUtil.clamp(d.value, 2, 17);
    }

    private FlywheelConstants() {}
}
