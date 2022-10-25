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

    public static final double kS = 0.2; // 0.57; // 0.74;
    public static final double kV = 0.55; // 0.2;
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
    public static final double kBatterVelocity = 9.5;
    private static final double kOffset = 0.41;
    private static final double kFarOffset = -0.4;

    public static final double[][] kFlywheelMap2 = {
        {Units.feetToMeters(2) + GlobalConstants.kCenterOffsetMeters, 12.5},
        {Units.feetToMeters(4) + GlobalConstants.kCenterOffsetMeters, 13.0 + kOffset},
        {Units.feetToMeters(6) + GlobalConstants.kCenterOffsetMeters, 14.5 + kOffset},
        {Units.feetToMeters(8) + GlobalConstants.kCenterOffsetMeters, 14.76 + kOffset},
        {Units.feetToMeters(10) + GlobalConstants.kCenterOffsetMeters, 15.2 + kOffset},
        {Units.feetToMeters(12) + GlobalConstants.kCenterOffsetMeters, 16.0 + kOffset},
        {Units.feetToMeters(14) + GlobalConstants.kCenterOffsetMeters, 17.0 + kOffset},
        {Units.feetToMeters(16) + GlobalConstants.kCenterOffsetMeters, 18.6 + kOffset},
        {Units.feetToMeters(18) + GlobalConstants.kCenterOffsetMeters, 21.0 + kOffset}
    };

    public static final double[][] kFlywheelVoltageMap = {
        {1.36, 1},
        {2.38, 1.5},
        {3.30, 2.0},
        {4.14, 2.5},
        {4.99, 3.0},
        {5.9, 3.5},
        {6.79, 4.0},
        {7.70, 4.5},
        {8.59, 5.0},
        {8.79, 5.1},
        {8.97, 5.2},
        {9.06, 5.25},
        {9.10, 5.26},
        {9.22, 5.3},
        {9.32, 5.4},
        {9.51, 5.5},
        {9.67, 5.6},
        {9.80, 5.65},
        {9.87, 5.7},
        {10.03, 5.8},
        {10.18, 5.85},
        {10.22, 5.9},
        {10.4, 6.0},
        {10.67, 6.1},
        {10.86, 6.2},
        {11.04, 6.3},
        {11.22, 6.4},
        {11.34, 6.5},
        {11.72, 6.6},
        {11.79, 6.7},
        {11.97, 6.8},
        {12.17, 6.9},
        {12.27, 7.0},
        {13.20, 7.5},
        {14.12, 8.0},
        {15.05, 8.5},
        {16.00, 9},
        {16.95, 9.5},
        {17.92, 10}
    };

    public static final double[][] kFlywheelMap = {
        {2.25, 9.3},
        {2.6, 9.5},
        {3, 9.6},
        {3.17, 9.7},
        {3.79, 10.1},
        {3.94, 10.4},
        {4.05, 10.5},
        {4.15, 10.6},
        {4.55, 11}
    };

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelAutoAimMap = new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelAutoAimMap2 = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelVelVoltage = new InterpolatingTreeMap<>();
    public static double constantVelocityMpS = 5;

    static {
        kMasterConfig.slot0.kP = 0.08; // 0.05;
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

        for (double[] pair : kFlywheelVoltageMap) {
            kFlywheelVelVoltage.put(
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
