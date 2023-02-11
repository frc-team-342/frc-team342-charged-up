package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** used for subsystems and devices that can have hardware connection checks and automatic test routines */
public interface Testable {

    /** represents status of hardware device connections */
    public class Connection {
        private String name; // device name
        private BooleanSupplier connectionCheck;

        /**
         * @param name device name to be displayed on dashboard in case of failure
         * @param connectionCheck function used to check whether device is connected
         */
        public Connection(String name, BooleanSupplier connectionCheck) {
            this.name = name; // device name for dashboard
            this.connectionCheck = connectionCheck; // function to check whether device is connected
        }

        /**
         * instantiate a connection with default name and check func from a sparkmax
         * known issue: will not update connection without restarting robot code
         * 
         * @param spark the sparkmax object
         * @return a {@link frc.robot.subsystems.Testable.Connection Connection} representing the state of the spark's CAN connection
         */
        public static Connection fromSparkMax(CANSparkMax spark) {
            int id = spark.getDeviceId(); // can id
            BooleanSupplier connectionCheck = () -> !spark.getFirmwareString().equals("v0.0.0"); 
            return new Connection("SparkMax " + Integer.toString(id), connectionCheck);       
        }

        /**
         * instantiate a connection with default name and check func from a navx
         * @param navx the navx object
         * @return a {@link frc.robot.subsystems.Testable.Connection Connection} representing the state of the navx's connection to the rio
         */
        public static Connection fromNavx(AHRS navx) {
            return new Connection("NavX", navx::isConnected);
        }

        /**
         * instantiate a connection with default name and check func from a limelight
         * known issue: will not update connection without restarting robot code
         * 
         * @param limelight the network table containing limelight data
         * @return a {@link frc.robot.subsystems.Testable.Connection Connection} representing the state of the limelight's network connection
         */
        public static Connection fromLimelight(NetworkTable limelight) {
            BooleanSupplier connectionCheck = () -> {
                // 4 is not a valid value of the led mode
                long ledMode = limelight.getEntry("ledMode").getInteger(4);
                return ledMode != (long) 4;
            };
            return new Connection("Limelight", connectionCheck);
        }

        /**
         * instantiate a connection with default name and check from a color sensor
         * @param colorSensor the rev color sensor v3 
         * @return a {@link frc.robot.subsystems.Testable.Connection Connection} representing the state of the color sensor's i2c connection
         */
        public static Connection fromColorSensor(ColorSensorV3 colorSensor) {
            return new Connection("Color Sensor", colorSensor::isConnected);
        }
        
        /**
         * @return the name of this device as it will be displayed on the dashboard
         */
        public String getName() { return name; }

        /**
         * @return whether or not device is connected, using the {@link java.util.function.BooleanSupplier BooleanSupplier} from the constructor
         */
        public boolean connected() { return connectionCheck.getAsBoolean(); }
    }
    
    /**
     * checks whether all the hardware in a subsystem is connected
     * @return list of connection objects representing each checkable hardware object in the subsystem
     */
    default public List<Connection> hardwareConnections() {
        // returns empty list when not implemented
        return List.of();
    }

    /**
     * test routine used for autonomous function checking
     * @return command with visual feedback to test function of this subsystem
     */
    default public CommandBase testRoutine() {
        // does nothing when not implemented
        return new InstantCommand();
    }

    /**
     * check every connection in the device list
     * @return if no failures, null, if failures present returns error string
     */
    default public String checkAllConnections() {
        // get all hardware connections for this subsystem
        List<Connection> connections = this.hardwareConnections();

        // used to concatenate errors from every device
        StringBuilder errors = new StringBuilder("");

        // check each connection in list
        for (Connection connection: connections) {
            if (connection.connected()) continue;

            // add error to stringbuilder
            errors.append(connection.getName() + " not connected.\n");
        }

        // return final error string after checking every device
        String errorMsg = errors.toString();
        return errorMsg;
    }

    
}
