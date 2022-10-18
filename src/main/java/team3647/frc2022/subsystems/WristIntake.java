// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import team3647.lib.PeriodicSubsystem;

public class WristIntake implements PeriodicSubsystem {
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;
    // private final ArmFeedforward degff;
    // private final SimpleMotorFeedforward degff;

    private final SimpleMotorFeedforward speedff;
    private final double kS;
    private final double kDt;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final double intakeVelocityConversion;
    private final double deployVelocityConversion;
    private final double deployPositionConversion;

    private final double maxDeployVel;

    private final double nominalVoltage;

    private final double intakableDeg;
    private final double zeroDeg;
    private final TrapezoidProfile.Constraints profileConstraints;

    public WristIntake(
            TalonFX deployMotor,
            TalonFX intakeMotor,
            // ArmFeedforward degff,
            double kS,
            // SimpleMotorFeedforward degff,
            SimpleMotorFeedforward speedff,
            TrapezoidProfile.Constraints profileConstraints,
            double kDt,
            double intakeVelocityConversion,
            double deployVelocityConversion,
            double deployPositionConversion,
            double nominalVoltage,
            double maxDeployVel,
            double intakableDeg,
            double zeroDeg) {
        // pid configured in constants for both
        this.deployMotor = deployMotor;
        this.intakeMotor = intakeMotor;

        this.kS = kS;

        this.maxDeployVel = maxDeployVel;

        // for deploy pos
        // this.degff = degff;
        // for intake speed
        this.speedff = speedff;

        this.intakeVelocityConversion = intakeVelocityConversion;
        this.deployVelocityConversion = deployVelocityConversion;
        this.deployPositionConversion = deployPositionConversion;
        this.intakableDeg = intakableDeg;
        this.zeroDeg = zeroDeg;
        this.kDt = kDt;
        this.profileConstraints = profileConstraints;
        this.nominalVoltage = nominalVoltage;
        this.deployMotor.setNeutralMode(NeutralMode.Brake);
        this.intakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    public static class PeriodicIO {
        // in degrees
        public double deployDeg = 0;
        // in deg/s
        public double deployVel = 0;
        // in m/s surface speed
        public double intakeVel = 0;

        public ControlMode intakeControlMode = ControlMode.Disabled;
        public ControlMode deployControlMode = ControlMode.Disabled;
        // in m/s
        public double intakeDemand = 0;
        public double intakeff = 0;

        // in deg
        public double deployDemand = 0;
        public double deployff = 0;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.deployDeg = deployMotor.getSelectedSensorPosition() * deployPositionConversion;
        periodicIO.deployVel = deployMotor.getSelectedSensorVelocity() * deployVelocityConversion;
        periodicIO.intakeVel = deployMotor.getSelectedSensorVelocity() * intakeVelocityConversion;
    }

    @Override
    public void writePeriodicOutputs() {
        intakeMotor.set(
                periodicIO.intakeControlMode,
                periodicIO.intakeDemand,
                DemandType.ArbitraryFeedForward,
                periodicIO.intakeff / nominalVoltage);

        deployMotor.set(
                periodicIO.deployControlMode,
                periodicIO.deployDemand,
                DemandType.ArbitraryFeedForward,
                periodicIO.deployff / nominalVoltage);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public double getDegrees() {
        return periodicIO.deployDeg;
    }

    public void extend(double deg) {
        this.setDegMotionProfile(this.intakableDeg, this.maxDeployVel);
    }

    public void extend() {
        // check sign
        extend(60.0);
    }

    // public void retract() {
    //     this.setDegMotionMagic(
    //             this.zeroDeg, degff.calculate(Units.degreesToRadians(zeroDeg), maxDeployVel));
    // }

    // vel in m/s surface vel
    public void setSurfaceVelocity(double vel) {
        periodicIO.intakeControlMode = ControlMode.Velocity;
        periodicIO.intakeff = speedff.calculate(getVelocity(), vel, kDt);
        periodicIO.intakeDemand = vel / intakeVelocityConversion;
    }

    // position in deg, ff in volts
    public void setDegMotionMagic(double positionDeg, double feedforward) {
        periodicIO.deployControlMode = ControlMode.MotionMagic;
        periodicIO.deployff = feedforward;
        // convert to set to native
        periodicIO.deployDemand = positionDeg / deployPositionConversion;
    }

    public void setDegMotionProfile(double positionDeg, double velocity) {
        periodicIO.deployControlMode = ControlMode.Position;
        TrapezoidProfile profile =
                new TrapezoidProfile(
                        this.profileConstraints,
                        new TrapezoidProfile.State(getDegrees(), getVelocity()),
                        new TrapezoidProfile.State(positionDeg, velocity));

        var state = profile.calculate(0.02);
        periodicIO.deployDemand = state.position / deployPositionConversion;
    }

    public void setOpenloop(double output) {
        periodicIO.intakeControlMode = ControlMode.PercentOutput;
        periodicIO.intakeDemand = output;
        periodicIO.intakeff = 0;
    }

    public double getVelocity() {
        return periodicIO.intakeVel;
    }

    public void resetEncoders() {
        deployMotor.setSelectedSensorPosition(0.0);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "wrist intake";
    }
}
