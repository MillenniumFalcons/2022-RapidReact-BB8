package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {
    private final double minPosDeg;
    private final double maxPosDeg;
    private final double posThresholdDeg;
    private final double kS;
    private final double kCos;

    private final double intakableDeg;
    private final double zeroDeg;

    public Wrist(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kS,
            double kCos,
            double kDt,
            double minPosDeg,
            double maxPosDeg,
            double posThresholdDeg,
            double intakableDeg,
            double zeroDeg) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.intakableDeg = intakableDeg;
        this.zeroDeg = zeroDeg;
        this.minPosDeg = minPosDeg;
        this.maxPosDeg = maxPosDeg;
        this.posThresholdDeg = posThresholdDeg;
        this.kS = kS;
        this.kCos = kCos;
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        resetEncoder();
        setToBrake();
    }

    public void retract() {
        setAngleMotionMagic(zeroDeg);
    }

    public void extend() {
        setAngleMotionMagic(intakableDeg);
    }

    public void setAngleMotionMagic(double angle) {
        double multiplier = Math.signum(angle - getAngle());
        // double gravityVoltage = Math.cos(Math.toRadians(angle - minPosDeg)) * kCos;

        // super.setPositionMotionMagic(
        //         MathUtil.clamp(angle, minPosDeg + posThresholdDeg, maxPosDeg - posThresholdDeg),
        //         kS * multiplier + gravityVoltage);
        super.setPositionMotionMagic(angle, multiplier * (-kS));
    }

    public double getAngle() {
        return super.getPosition();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
