package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Column;

public class ColumnCommands {
    private final Column column;

    public ColumnCommands(Column column) {
        this.column = column;
    }

    public Command getGoVariableVelocity(DoubleSupplier surfaceVelMpSFunction) {
        return new FunctionalCommand(
                () -> {},
                () -> column.setSurfaceVelocity(surfaceVelMpSFunction.getAsDouble()),
                interrupted -> column.setOpenloop(0),
                () -> false,
                column);
    }

    public Command getRunInwards() {
        return new FunctionalCommand(
                () -> {},
                () -> column.setOpenloop(0.3),
                interrupted -> column.setOpenloop(0),
                () -> false,
                column);
    }

    public Command getRunOutwards() {
        return new FunctionalCommand(
                () -> {},
                () -> column.setOpenloop(-0.3),
                interrupted -> column.setOpenloop(0),
                () -> false,
                column);
    }

    public Command getRunInwardsUntil(BooleanSupplier interruptOn) {
        return getRunInwards().withInterrupt(interruptOn);
    }

    public Command getRunOutwardsUntil(BooleanSupplier interruptOn) {
        return getRunOutwards().withInterrupt(interruptOn);
    }

    public Command getEndSequence() {
        return new RunCommand(column::end, column);
    }
}
