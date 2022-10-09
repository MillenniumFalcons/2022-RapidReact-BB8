package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Column;

public class ColumnCommands {
    private final Column columnTop;

    public ColumnCommands(Column columnTop) {
        this.columnTop = columnTop;
    }

    public Command getGoVariableVelocity(DoubleSupplier surfaceVelMpSFunction) {
        return new FunctionalCommand(
                () -> {},
                () -> columnTop.setSurfaceVelocity(surfaceVelMpSFunction.getAsDouble()),
                interrupted -> columnTop.setOpenloop(0),
                () -> false,
                columnTop);
    }

    public Command getRunInwards() {
        return new FunctionalCommand(
                () -> {},
                () -> columnTop.setOpenloop(0.3),
                interrupted -> columnTop.setOpenloop(0),
                () -> false,
                columnTop);
    }

    public Command getRunOutwards() {
        return new FunctionalCommand(
                () -> {},
                () -> columnTop.setOpenloop(-0.3),
                interrupted -> columnTop.setOpenloop(0),
                () -> false,
                columnTop);
    }

    public Command getRunInwardsUntil(BooleanSupplier interruptOn) {
        return getRunInwards().withInterrupt(interruptOn);
    }

    public Command getRunOutwardsUntil(BooleanSupplier interruptOn) {
        return getRunOutwards().withInterrupt(interruptOn);
    }

    public Command getEndSequence() {
        return new RunCommand(columnTop::end, columnTop);
    }
}
