package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Testable;

class Limelight implements Testable {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public int getPipeline() {
        return table.getEntry("pipeline").getNumber(0).intValue();
    }

    public void togglePipeline() {
        int currPipe = getPipeline();
        if (currPipe == 0) {
            table.getEntry("pipeline").setNumber(1);
        } else {
            table.getEntry("pipeline").setNumber(0);
        }
    }

    /**
     * change the led mode of the limelight
     * @param mode 0 for pipeline default, 1 for off, 2 for blink, 3 for on
     */
    public void setLedMode(int mode) {
        table.getEntry("ledMode").setInteger(mode);
    }

    @Override
    public List<Connection> hardwareConnections() {
        return List.of(
            Connection.fromLimelight(table)
        );
    }

    @Override
    public CommandBase testRoutine() {
        return Commands.sequence(
            // set leds to blink
            new InstantCommand(
                () -> { setLedMode(2); }
            ),

            // wait for 2 seconds
            new WaitCommand(2),
            
            // set leds back to pipeline default
            new InstantCommand(
                () -> { setLedMode(0); }
            )
        );
    }
}