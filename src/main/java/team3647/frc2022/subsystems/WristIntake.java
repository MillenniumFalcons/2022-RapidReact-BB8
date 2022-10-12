// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
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
        System.out.println("Demand : " + periodicIO.deployDemand);
        System.out.println("FF : " + periodicIO.deployff);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public double getDegrees() {
        return periodicIO.deployDeg;
    }

    // public void extend(double deg) {
    //     setDegMotionMagic(deg, degff.calculate(Units.degreesToRadians(intakableDeg),
    // maxDeployVel));
    //     System.out.println("Here: " + periodicIO.deployDeg);
    //     System.out.println(
    //             "There: " + degff.calculate(Units.degreesToRadians(intakableDeg), maxDeployVel));
    // }

    public void extend(double deg) {
        this.setAngleMotionMagic(deg);
    }

    public void extend() {
        // check sign
        extend(60.0);
    }

    public void increaseDemand() {
        periodicIO.deployDemand += 100;
    }

    public void increaseFF() {
        periodicIO.deployff += 0.1;
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

    public void setAngleMotionMagic(double angle) {
        double multiplier = Math.signum(angle - getDegrees());
        periodicIO.deployDemand = MathUtil.clamp(angle, -10, 180) / deployPositionConversion;
        periodicIO.deployff = kS * multiplier;
    }

    // position in deg, ff in volts
    public void setDegMotionMagic(double positionDeg, double feedforward) {
        periodicIO.deployControlMode = ControlMode.MotionMagic;
        periodicIO.deployff = feedforward;
        // convert to set to native
        periodicIO.deployDemand = positionDeg / deployPositionConversion;
    }

    public void setDemanOpenLoop(double openLoop) {
        periodicIO.deployControlMode = ControlMode.PercentOutput;
        periodicIO.deployDemand = openLoop;
        periodicIO.deployff = 0;
    }

    public void setDegMotionProfile(double positionDeg, double velocity) {
        periodicIO.deployControlMode = ControlMode.MotionMagic;
        TrapezoidProfile profile =
                new TrapezoidProfile(
                        this.profileConstraints,
                        new TrapezoidProfile.State(getDegrees(), getVelocity()),
                        new TrapezoidProfile.State(positionDeg, velocity));

        var state = profile.calculate(0.02);
        // Multiply the static friction volts by -1 if our target position is less than current
        // position; if we need to move backwards, the volts needs to be negative
        // double ffVolts = degff.calculate(state.position, state.velocity);
        // if (Math.abs(ffVolts) < 0.000001) {
        //     ffVolts = kS * Math.signum(state.position - getPosition());
        // }

        // System.out.println("Position: " + state.position);
        // System.out.println("Velocity: " + state.velocity);
        // periodicIO.deployff = ffVolts;
        periodicIO.deployDemand = state.position;
        System.out.println("FF : " + periodicIO.deployff);
        System.out.println("Demand : " + periodicIO.deployDemand);
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
