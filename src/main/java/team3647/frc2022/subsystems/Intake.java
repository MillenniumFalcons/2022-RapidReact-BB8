package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Intake extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;

    public Intake(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.ff = ff;
    }

    public void setSurfaceVelocity(double vel) {
        setVelocity(vel, ff.calculate(getVelocity(), vel, kDt));
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
