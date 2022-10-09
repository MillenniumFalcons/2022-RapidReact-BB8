package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.drivers.LazyTalonFX;

public final class ColumnConstants {
    public static final TalonFXInvertType kMasterInverted = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kS = 0.89015;
    public static final double kV = 1;
    public static final double kA = 0.026327;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kNominalVoltage = 12;
    public static final double kGearboxReduction = 24 / 31.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(2);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;

    public static final double kPosConverstion =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;
    public static final double kNativeVelToSurfaceMpS =
            10 * kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    public static final TalonFX kColumnMotor =
            new LazyTalonFX(GlobalConstants.ColumnIds.kMasterMotorId);
    public static final DigitalInput kTopBanner =
            new DigitalInput(GlobalConstants.ColumnIds.kTopSensorPin);

    public static final double kBatterVelocity = 5;
    public static final double kLowGoalVelocity = 3;

    static {
        kMasterConfig.slot0.kP = 0.1;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;

        kColumnMotor.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
    }
}
