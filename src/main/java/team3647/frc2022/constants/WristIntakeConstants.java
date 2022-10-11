// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;

/** Add your docs here. */
public class WristIntakeConstants {
    public static final InvertType kIntakeMotorInverted = InvertType.InvertMotorOutput;
    public static final InvertType kDeployMotorInverted = InvertType.None;
    public static final TalonFXConfiguration kIntakeMotorConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kDeployMotorConfig = new TalonFXConfiguration();

    public static final double intakableDegree = 60.0;
    public static final double zeroDeg = 0.0;
    public static final double maxDeployVelocityDegPerSec = Units.degreesToRadians(40.0);

    public static final double kIntakeS = 0.75412;
    public static final double kIntakeV = 0.72691;
    public static final double kIntakeA = 0.020019;
    public static final SimpleMotorFeedforward kIntakeFeedForward =
            new SimpleMotorFeedforward(kIntakeS, kIntakeV, kIntakeA);

    public static final double kDeployS = 8.85;
    public static final double kDeployG = 1.013;
    public static final double kDeployA = 0.0032323;
    public static final ArmFeedforward kDeployFeedForward =
            new ArmFeedforward(kDeployS, kDeployG, kDeployA);

    public static final double kNominalVoltage = 10.0;

    public static final TalonFX kIntakeMotor =
            new LazyTalonFX(GlobalConstants.IntakeIds.kIntakeMotorId);
    public static final TalonFX kDeployMotor =
            new LazyTalonFX(GlobalConstants.IntakeIds.kDeployMotorId);

    public static final double kWheelDiameterMeters = 0.0762;

    public static final double kDeployGearboxReduction = 1.0 / 35.0;
    public static final double kIntakeGearboxReduction = 1.0 / 5.33;

    public static final double kIntakeWheelRotationToMeters =
            kWheelDiameterMeters * Math.PI * kIntakeGearboxReduction;

    public static final double kDeployNativePositionToDegrees =
            kDeployGearboxReduction / 2048.0 * 360;
    public static final double kDeployNativeVelocityToDegpS = kDeployNativePositionToDegrees * 10;

    public static final double kIntakeNativeVelToSurfaceMpS =
            10 * kIntakeWheelRotationToMeters / GlobalConstants.kFalconTicksPerRotation;

    static {
        kIntakeMotorConfig.slot0.kP = 0.1;
        kIntakeMotorConfig.slot0.kI = 0;
        kIntakeMotorConfig.slot0.kD = 0;
        kIntakeMotorConfig.slot0.kF = 0;

        kDeployMotorConfig.slot0.kP = 1.5;
        kDeployMotorConfig.slot0.kI = 0;
        kDeployMotorConfig.slot0.kD = 0;
        kDeployMotorConfig.slot0.kF = 0;

        kIntakeMotorConfig.voltageCompSaturation = kNominalVoltage;
        kDeployMotorConfig.voltageCompSaturation = kNominalVoltage;

        kIntakeMotor.configAllSettings(kIntakeMotorConfig, GlobalConstants.kTimeoutMS);
        kDeployMotor.configAllSettings(kDeployMotorConfig, GlobalConstants.kTimeoutMS);
        kIntakeMotor.setInverted(kIntakeMotorInverted);
        kDeployMotor.setInverted(kDeployMotorInverted);
    }

    private WristIntakeConstants() {}
}
