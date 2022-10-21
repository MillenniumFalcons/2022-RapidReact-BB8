package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.TalonFXSubsystem;

public class Flywheel extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;

    public Flywheel(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            TalonFX follower,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.addFollower(follower, FollowerType.PercentOutput, InvertType.OpposeMaster);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        setToCoast();
        this.ff = ff;
    }

    /** @param vel velocity in m/s, positive is outside (to shoot) */
    public void setSurfaceSpeed(double vel) {
        setVelocity(vel, ff.calculate(vel));
    }

    @Override
    public void writePeriodicOutputs() {
        super.writePeriodicOutputs();
        SmartDashboard.putNumber("velocity", getVelocity());
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}
