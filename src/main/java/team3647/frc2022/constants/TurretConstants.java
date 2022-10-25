package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;

public class TurretConstants {

    public static final TalonFX kTurretMotor = new LazyTalonFX(GlobalConstants.TurretIds.kMotorId);
    public static final TalonFXInvertType kTurretMotorInvert = TalonFXInvertType.CounterClockwise;

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kGearboxReduction = 16 / 60.0 * 16 / 150.0;
    public static final double kFalconPositionToDegrees = kGearboxReduction / 2048.0 * 360;
    public static final double kFalconVelocityToDegpS = kFalconPositionToDegrees * 10;
    public static final double kMaxDegree = 0; // 120;
    public static final double kMinDegree = -270; // -210;
    public static final double kPosThersholdDeg = 1.0;
    public static final boolean kCurrentLimitingEnable = true;

    public static final double kS = 0;
    public static final double kV = 0.011614;
    public static final double kA = 0.005;

    public static final double kMaxVelocityDegPs = 45;
    public static final double kMaxAccelerationDegPss = 45;

    public static final TrapezoidProfile.Constraints kTurretProfile =
            new TrapezoidProfile.Constraints(kMaxVelocityDegPs, kMaxAccelerationDegPss);

    public static final double kMaxVelocityTicks = kMaxVelocityDegPs / kFalconVelocityToDegpS;
    public static final double kMaxAccelerationTicks =
            kMaxAccelerationDegPss / kFalconVelocityToDegpS;

    public static final double kStallCurrent = 10;
    public static final double kContinuousCurrentLimit = 35; // amps
    public static final double kPeakCurrentLimit = 40; // amps
    public static final double kPeakCurrentDuration = 10; // milliseconds
    public static final double kNominalVoltage = 11;

    public static final Translation2d kRobotToTurretFixed =
            new Translation2d(Units.inchesToMeters(2), 0);
    public static final Translation2d kTurretToCamTranslationMeters =
            new Translation2d(Units.inchesToMeters(10.37), 0);
    public static final Pose2d kTurretToCamFixed =
            new Pose2d(kTurretToCamTranslationMeters, new Rotation2d());

    public static final Transform2d kTurretToCamFixedTransform =
            new Transform2d(kTurretToCamTranslationMeters, new Rotation2d());

    public static final SimpleMotorFeedforward kFeedForwards =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static final double kStartingAngle = -90;

    static {
        kMasterConfig.slot0.kP = 0.3;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.slot1.kP = 0.3;
        kMasterConfig.slot1.kI = 0;
        kMasterConfig.slot1.kD = 0;
        kMasterConfig.slot1.kF = kV / kNominalVoltage * kFalconVelocityToDegpS * 1023;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMasterConfig.supplyCurrLimit.enable = kCurrentLimitingEnable;
        kMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kPeakCurrentLimit;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = kPeakCurrentDuration;
        kMasterConfig.peakOutputForward = 0.7;
        kMasterConfig.peakOutputReverse = -0.7;
        // kMasterConfig.closedloopRamp = 1;
        // kMasterConfig.peakOutputForward = 0.5;
        // kMasterConfig.peakOutputReverse = -0.5;

        // in native units/100ms/s
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        // in native units/100ms
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;

        kTurretMotor.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kTurretMotor.setInverted(kTurretMotorInvert);
    }
}
