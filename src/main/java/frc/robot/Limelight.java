package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

class Limelight {

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
}