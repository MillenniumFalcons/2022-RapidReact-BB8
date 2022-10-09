package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Flywheel;

public class FlywheelCommands {
    private final Flywheel m_flywheel;

    public FlywheelCommands(Flywheel flywheel) {
        this.m_flywheel = flywheel;
    }

    public Command constantVelocity(double surfaceVelMpS) {
        return variableVelocity(() -> surfaceVelMpS);
    }

    public Command variableVelocity(DoubleSupplier surfaceVelMpSFunction) {
        return new FunctionalCommand(
                () -> {},
                () -> m_flywheel.setSurfaceSpeed(surfaceVelMpSFunction.getAsDouble()),
                interrupted -> m_flywheel.setOpenloop(0),
                () -> false,
                m_flywheel);
    }

    public Command waitToSpinDownThenHold(double velToKeep) {
        return waitToSpinDownThenHold(() -> velToKeep);
    }

    public Command waitToSpinDownThenHold(DoubleSupplier velToKeep) {
        return new WaitUntilCommand(() -> m_flywheel.getVelocity() < velToKeep.getAsDouble())
                .deadlineWith(new RunCommand(() -> m_flywheel.setOpenloop(0), m_flywheel))
                .andThen(
                        new RunCommand(
                                () -> m_flywheel.setSurfaceSpeed(velToKeep.getAsDouble()),
                                m_flywheel));
    }

    public Command openloop(DoubleSupplier demand) {
        return new FunctionalCommand(
                () -> {},
                () -> m_flywheel.setOpenloop(demand.getAsDouble()),
                interrupted -> m_flywheel.setOpenloop(0),
                () -> false,
                m_flywheel);
    }

    public Command openloop(double demand) {
        return new FunctionalCommand(
                () -> {},
                () -> m_flywheel.setOpenloop(demand),
                interrupted -> m_flywheel.setOpenloop(0),
                () -> false,
                m_flywheel);
    }

    /**
     * This command will not stop the flywheel when it ends (use for accelerate before shooting), it
     * ends once we reach the speed
     *
     * @param surfaceVelMpSFunction
     * @return
     */
    public Command accelerate(DoubleSupplier surfaceVelMpSFunction) {
        return new FunctionalCommand(
                () -> {},
                () -> m_flywheel.setSurfaceSpeed(surfaceVelMpSFunction.getAsDouble()),
                interrupted -> {},
                () -> m_flywheel.getVelocity() > surfaceVelMpSFunction.getAsDouble() - 0.1,
                m_flywheel);
    }

    public Command stop() {
        return new RunCommand(() -> m_flywheel.setOpenloop(0));
    }
}
