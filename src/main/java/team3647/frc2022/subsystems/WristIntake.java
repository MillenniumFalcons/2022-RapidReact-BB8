// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import team3647.lib.PeriodicSubsystem;

public class WristIntake implements PeriodicSubsystem {
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;
    private final ArmFeedforward degff;
    private final SimpleMotorFeedforward speedff;
    private final double kDt;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final double intakeVelocityConversion;
    private final double deployVelocityConversion;
    private final double deployPositionConversion;

    private final double maxDeployVel;

    private final double nominalVoltage;

    private final double intakableDeg;

    public WristIntake(
            TalonFX deployMotor,
            TalonFX intakeMotor,
            ArmFeedforward degff,
            SimpleMotorFeedforward speedff,
            double kDt,
            double intakeVelocityConversion,
            double deployVelocityConversion,
            double deployPositionConversion,
            double nominalVoltage,
            double maxDeployVel,
            double intakableDeg) {
        // pid configured in constants for both
        this.deployMotor = deployMotor;
        this.intakeMotor = intakeMotor;

        this.maxDeployVel = maxDeployVel;

        // for deploy pos
        this.degff = degff;
        // for intake speed
        this.speedff = speedff;

        this.intakeVelocityConversion = intakeVelocityConversion;
        this.deployVelocityConversion = deployVelocityConversion;
        this.deployPositionConversion = deployPositionConversion;
        this.intakableDeg = intakableDeg;
        this.kDt = kDt;
        this.nominalVoltage = nominalVoltage;
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

    public void extend() {
        // check sign
        setDegMotionMagic(
                intakableDeg, degff.calculate(Units.degreesToRadians(intakableDeg), maxDeployVel));
    }

    public void retract() {
        double zeroDeg = 10;
        setDegMotionMagic(zeroDeg, degff.calculate(Units.degreesToRadians(zeroDeg), maxDeployVel));
    }

    // vel in m/s surface vel
    public void setSurfaceVelocity(double vel) {
        periodicIO.intakeControlMode = ControlMode.Velocity;
        periodicIO.intakeff = speedff.calculate(getVelocity(), vel, kDt);
        periodicIO.intakeDemand = vel / intakeVelocityConversion;
    }

    // position in deg, ff in volts
    private void setDegMotionMagic(double position, double feedforward) {
        periodicIO.deployControlMode = ControlMode.MotionMagic;
        periodicIO.deployff = feedforward;
        // convert to set to native
        periodicIO.deployDemand = position / deployPositionConversion;
    }

    public void setOpenloop(double output) {
        periodicIO.intakeControlMode = ControlMode.PercentOutput;
        periodicIO.intakeDemand = output;
        periodicIO.intakeff = 0;
    }

    public double getVelocity() {
        return periodicIO.intakeVel;
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "wrist intake";
    }
}
