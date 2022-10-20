package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import team3647.lib.TalonFXSubsystem;

public class Turret extends TalonFXSubsystem {

    private final double maxAngle;
    private final double minAngle;
    private final SimpleMotorFeedforward ff;
    private final double kS;
    private final TrapezoidProfile.Constraints profileContraints;
    private TrapezoidProfile profile;
    private double prevAngle;
    private double prevVelocity;
    private double startTime;

    public Turret(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double kS,
            double maxAngle,
            double minAngle,
            double startingAngle,
            TrapezoidProfile.Constraints profileConstraints,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.ff = ff;
        this.kS = kS;
        this.profileContraints = profileConstraints;
        profile = new TrapezoidProfile(profileConstraints, new TrapezoidProfile.State());
        setEncoder(startingAngle);
        prevAngle = startingAngle;
        prevVelocity = 0;
        startTime = Timer.getFPGATimestamp();
        setToBrake();
    }

    /** @param angle in degree, [-180,180] */
    public void setAngle(double angle, double velocity) {
        angle -= 360.0 * Math.round(angle / 360.0); // angles in [-180, 180]
        if (angle < minAngle) {
            angle += 360;
        } else if (angle > maxAngle) {
            angle -= 360;
        }
        profile =
                new TrapezoidProfile(
                        this.profileContraints,
                        new TrapezoidProfile.State(getPosition(), getVelocity()),
                        new TrapezoidProfile.State(angle, velocity));

        if (angle != prevAngle || Math.abs(prevVelocity - velocity) > 0.001) {
            startTime = Timer.getFPGATimestamp();
        }
        var state = profile.calculate(0.02);
        // Multiply the static friction volts by -1 if our target position is less than current
        // position; if we need to move backwards, the volts needs to be negative
        double ffVolts = ff.calculate(state.velocity);
        // if (Math.abs(ffVolts) < 0.000001) {
        //     ffVolts = kS * Math.signum(state.position - getPosition());
        // }

        // System.out.println("Position: " + state.position);
        // System.out.println("Velocity: " + state.velocity);

        setPosition(state.position, ffVolts);
    }

    public void setAngleMotionMagic(double angle) {
        angle = MathUtil.clamp(angle, minAngle, maxAngle);
        setPositionMotionMagic(angle, 0);
        // throw new UnsupportedOperationException("Method unimplemented yet");
    }

    /** @return angle in [-180,180] */
    public double getAngle() {
        double angle = getPosition();
        angle -= 360.0 * Math.round(angle / 360.0);
        return angle;
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    @Override
    public String getName() {
        return "Turret";
    }
}
