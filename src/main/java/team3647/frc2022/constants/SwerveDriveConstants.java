package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team3647.frc2022.subsystems.SwerveModule;
import team3647.lib.drivers.LazyTalonFX;

public class SwerveDriveConstants {
    // default falcon rotates counter clockwise (CCW)

    public static final boolean canCoderInvert = false;
    public static final boolean kDriveMotorInverted = true;
    public static final boolean kTurnMotorInverted = true;

    // physical possible max speed
    public static final double kDrivePossibleMaxSpeedMPS = 4.5; // 4.5;
    public static final double kRotPossibleMaxSpeedRadPerSec = 10; // 10;

    public static final NeutralMode kTurnNeutralMode = NeutralMode.Coast;
    public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

    public static final TalonFX kFrontLeftDrive =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftDriveId);
    public static final TalonFX kFrontLeftTurn =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftTurnId);
    public static final CANCoder kFrontLeftAbsEncoder =
            new CANCoder(GlobalConstants.SwerveDriveIds.kFrontLeftAbsEncoderPort);

    public static final TalonFX kFrontRightDrive =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kFrontRightDriveId);
    public static final TalonFX kFrontRightTurn =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kFrontRightTurnId);
    public static final CANCoder kFrontRightAbsEncoder =
            new CANCoder(GlobalConstants.SwerveDriveIds.kFrontRightAbsEncoderPort);

    public static final TalonFX kBackLeftDrive =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kBackLeftDriveId);
    public static final TalonFX kBackLeftTurn =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kBackLeftTurnId);
    public static final CANCoder kBackLeftAbsEncoder =
            new CANCoder(GlobalConstants.SwerveDriveIds.kBackLeftAbsEncoderPort);

    public static final TalonFX kBackRightDrive =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kBackRightDriveId);
    public static final TalonFX kBackRightTurn =
            new LazyTalonFX(GlobalConstants.SwerveDriveIds.kBackRightTurnId);
    public static final CANCoder kBackRightAbsEncoder =
            new CANCoder(GlobalConstants.SwerveDriveIds.kBackRightAbsEncoderPort);

    public static final Pigeon2Configuration kGyroConfig = new Pigeon2Configuration();

    // config swerve module reversed here, module class doens't reverse for you

    // distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21.75);
    // distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21.75);
    // translations are locations of each module wheel
    // 0 --> ++ --> front left
    // 1 --> +- --> front right
    // 2 --> -+ --> back left
    // 3 --> -- --> back right
    // c is center of robot,
    // +x towards front of robot, +y towards left of robot
    //         +x
    //         ^
    //         |
    //    +y<--c
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    //  config conversion factors here for each module. in meters for postiion and radians for
    // rotation.

    // from motor to output shaft
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    //     public static final double kTurnMotorGearRatio = 15.0 / 32.0 * 10.0 / 60.0;
    public static final double kTurnMotorGearRatio = 7.0 / 150.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);

    //     public static final double kWheelRotationToMetersDrive =
    //             kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

    //     public static final double kDriveMotorNativeToMeter =
    //             kWheelRotationToMetersDrive
    //                     / GlobalConstants.kFalconTicksPerRotation; // meters / native

    //     // divide for tick to deg
    public static final double kTurnMotorNativeToDeg =
            kTurnMotorGearRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;
    //     // deg / native
    //     public static final double kDriveMotorNativeToMPS =
    //             kWheelRotationToMetersDrive
    //                     * 10
    //                     / GlobalConstants.kFalconTicksPerRotation; // MPS / Native/10ms
    public static final double kTurnMotorNativeToDPS =
            kTurnMotorNativeToDeg * 10.0; // RPS / Native/10ms

    public static final double kWheelRotationToMetersDrive =
            kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS =
            kWheelRotationToMetersDrive * 10.0 / GlobalConstants.kFalconTicksPerRotation;

    public static final double kFalconTicksToMeters =
            kWheelRotationToMetersDrive / GlobalConstants.kFalconTicksPerRotation;

    public static final double kNominalVoltage = 10;
    public static final double kStallCurrent = 35;
    public static final double kMaxCurrent = 60;

    // find
    public static final double kAbsFrontLeftEncoderOffsetDeg = 32.6953125; // 35.66 - 2.8767;
    public static final double kAbsFrontRightEncoderOffsetDeg = 322.6660 + 9.5; // 333.28 + 0.0866;
    public static final double kAbsBackLeftEncoderOffsetDeg = 36.386718; // 36.39 - 0.1725;
    public static final double kAbsBackRightEncoderOffsetDeg = 146.886; // 148.97 + 0.8740;

    // max speed limits that we want
    public static final double kTeleopDriveMaxAccelUnitsPerSec = kDrivePossibleMaxSpeedMPS / 2;
    public static final double kTeleopDriveMaxAngularAccelUnitsPerSec =
            kRotPossibleMaxSpeedRadPerSec / 3;

    public static final TalonFXConfiguration kFrontLeftDriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kFrontLeftTurnConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kFrontRightDriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kFrontRightTurnConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kBackLeftDriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kBackLeftTurnConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kBackRightDriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kBackRightTurnConfig = new TalonFXConfiguration();

    public static final CANCoderConfiguration kMasterCancoderConfig = new CANCoderConfiguration();

    // master FF for drive for all modules
    public static final double kS = (0.56744 / 12); // 0.56744; // Volts
    public static final double kV = (2.5 / 12.0); // Volts
    public static final double kA = (0.0 / 12); // Volts

    public static final SimpleMotorFeedforward kMasterDriveFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    // master PID constants for turn and drive for all modules
    public static final double kDriveP = 0.1; // 1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double kTurnP = 0.8;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 12.0; // 1;

    // is stored as reference?
    public static final SwerveModule kFrontLeftModule =
            new SwerveModule(
                    kFrontLeftDrive,
                    kFrontLeftTurn,
                    kMasterDriveFeedforward,
                    kFrontLeftAbsEncoder,
                    kAbsFrontLeftEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);
    public static final SwerveModule kFrontRightModule =
            new SwerveModule(
                    kFrontRightDrive,
                    kFrontRightTurn,
                    kMasterDriveFeedforward,
                    kFrontRightAbsEncoder,
                    kAbsFrontRightEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);
    public static final SwerveModule kBackLeftModule =
            new SwerveModule(
                    kBackLeftDrive,
                    kBackLeftTurn,
                    kMasterDriveFeedforward,
                    kBackLeftAbsEncoder,
                    kAbsBackLeftEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);
    public static final SwerveModule kBackRightModule =
            new SwerveModule(
                    kBackRightDrive,
                    kBackRightTurn,
                    kMasterDriveFeedforward,
                    kBackRightAbsEncoder,
                    kAbsBackRightEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);

    public static final Pigeon2 kGyro = new Pigeon2(GlobalConstants.SwerveDriveIds.gyroPin);

    private static void setTurnMotorConfig(TalonFXConfiguration config) {
        config.slot0.kP = kTurnP;
        config.slot0.kI = kTurnI;
        config.slot0.kD = kTurnD;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = kStallCurrent;
        config.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        config.voltageCompSaturation = kNominalVoltage;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
    }

    private static void setDriveMotorConfig(TalonFXConfiguration config) {
        config.slot0.kP = kDriveP;
        config.slot0.kI = kDriveI;
        config.slot0.kD = kDriveD;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = kStallCurrent;
        config.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.voltageCompSaturation = kNominalVoltage;
    }

    private static void setCancoderConfig(CANCoderConfiguration config) {
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    static {
        kGyro.configFactoryDefault();
        kGyroConfig.ZAxisGyroError = 0.3;
        kGyro.configAllSettings(kGyroConfig);
        // encoder feedback alredy continous for turn motor?
        kFrontLeftDrive.configFactoryDefault();
        kFrontRightDrive.configFactoryDefault();
        kBackLeftDrive.configFactoryDefault();
        kBackRightDrive.configFactoryDefault();

        kFrontLeftTurn.configFactoryDefault();
        kFrontRightTurn.configFactoryDefault();
        kBackLeftTurn.configFactoryDefault();
        kBackRightTurn.configFactoryDefault();

        setTurnMotorConfig(kFrontLeftTurnConfig);
        setTurnMotorConfig(kFrontRightTurnConfig);
        setTurnMotorConfig(kBackLeftTurnConfig);
        setTurnMotorConfig(kBackRightTurnConfig);

        setDriveMotorConfig(kFrontLeftDriveConfig);
        setDriveMotorConfig(kFrontRightDriveConfig);
        setDriveMotorConfig(kBackLeftDriveConfig);
        setDriveMotorConfig(kBackRightDriveConfig);

        kFrontLeftAbsEncoder.configFactoryDefault();
        kFrontRightAbsEncoder.configFactoryDefault();
        kBackLeftAbsEncoder.configFactoryDefault();
        kBackRightAbsEncoder.configFactoryDefault();

        setCancoderConfig(kMasterCancoderConfig);
        kFrontLeftAbsEncoder.configAllSettings(kMasterCancoderConfig);
        kFrontRightAbsEncoder.configAllSettings(kMasterCancoderConfig);
        kBackLeftAbsEncoder.configAllSettings(kMasterCancoderConfig);
        kBackRightAbsEncoder.configAllSettings(kMasterCancoderConfig);

        kFrontLeftTurn.configAllSettings(kFrontLeftTurnConfig, GlobalConstants.kTimeoutMS);
        kFrontRightTurn.configAllSettings(kFrontRightTurnConfig, GlobalConstants.kTimeoutMS);
        kBackLeftTurn.configAllSettings(kBackLeftTurnConfig, GlobalConstants.kTimeoutMS);
        kBackRightTurn.configAllSettings(kBackRightTurnConfig, GlobalConstants.kTimeoutMS);

        kFrontLeftDrive.configAllSettings(kFrontLeftDriveConfig, GlobalConstants.kTimeoutMS);
        kFrontRightDrive.configAllSettings(kFrontRightDriveConfig, GlobalConstants.kTimeoutMS);
        kBackLeftDrive.configAllSettings(kBackLeftDriveConfig, GlobalConstants.kTimeoutMS);
        kBackRightDrive.configAllSettings(kBackRightDriveConfig, GlobalConstants.kTimeoutMS);

        // set invert, same for all b/c all motors facing same direction
        kFrontLeftTurn.setInverted(kTurnMotorInverted);
        kFrontRightTurn.setInverted(kTurnMotorInverted);
        kBackLeftTurn.setInverted(kTurnMotorInverted);
        kBackRightTurn.setInverted(kTurnMotorInverted);

        kFrontLeftDrive.setInverted(kDriveMotorInverted);
        kFrontRightDrive.setInverted(kDriveMotorInverted);
        kBackLeftDrive.setInverted(kDriveMotorInverted);
        kBackRightDrive.setInverted(kDriveMotorInverted);
    }

    private SwerveDriveConstants() {}
}
