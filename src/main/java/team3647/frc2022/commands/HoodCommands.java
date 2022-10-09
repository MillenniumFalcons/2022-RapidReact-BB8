package team3647.frc2022.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Hood;

public class HoodCommands {
    private final Hood m_hood;

    public HoodCommands(Hood hood) {
        this.m_hood = hood;
    }

    public Command zero() {
        MedianFilter hiVelFilter = new MedianFilter(10);
        MedianFilter loVelFilter = new MedianFilter(10);

        return new FunctionalCommand(
                        () -> hiVelFilter.reset(),
                        () -> m_hood.setOpenloop(-0.2),
                        interrupted -> {},
                        () -> hiVelFilter.calculate(m_hood.getVelocity()) > 50,
                        m_hood)
                .andThen(
                        new FunctionalCommand(
                                () -> loVelFilter.reset(),
                                () -> m_hood.setOpenloop(-0.2),
                                interrupted -> {
                                    if (!interrupted) {
                                        m_hood.resetEncoder();
                                    }
                                    m_hood.setOpenloop(0);
                                },
                                () -> Math.abs(loVelFilter.calculate(m_hood.getVelocity())) < 0.01,
                                m_hood));
    }

    public Command autoAdjustAngle(DoubleSupplier hoodAngle) {
        return new RunCommand(() -> m_hood.setAngleMotionMagic(hoodAngle.getAsDouble()), m_hood);
    }

    public Command motionMagic(double setpointDegrees) {
        return new FunctionalCommand(
                () -> {},
                () -> m_hood.setAngleMotionMagic(setpointDegrees),
                interrupted -> {},
                () -> Math.abs(m_hood.getAngle() - setpointDegrees) < 0.05,
                m_hood);
    }
}
