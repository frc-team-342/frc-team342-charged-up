package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


class Limelight {

    /*
     * Creates a network table instance and an entry for each necessary value
     */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry targets = table.getEntry("tv");
    private NetworkTableEntry horizontalOffset = table.getEntry("tx");
    private NetworkTableEntry verticalOffset = table.getEntry("ty");
    private NetworkTableEntry targetArea = table.getEntry("ta");
    private NetworkTableEntry robotPosition3D = table.getEntry("camtran");
    private NetworkTableEntry robotSkew = table.getEntry("ts0");

    /*
     * Creates a default array of values for use with the cam-tran table entry
     */
    private Number[] defaultValues =
    {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };


    /*
     * Creates a number Array to store the robot position values from the cam-tran entry
     * Fills it with the numbers from the defaultValues array
     */
    private Number[] robotPositionValues = getRobotPosition3D().getNumberArray(defaultValues);


    /*
     * Gets the current pipeline of the limelight
     * Outputs either 0 or 1: 0 is for RR tape and 1 is for Apriltags
     */
    public int getPipeline() {

        return table.getEntry("pipeline").getNumber(0).intValue();

    }


    /*
     * Switches between the retroreflective tape & Apriltag pipelines
     */
    public void togglePipeline() {

        int currPipe = getPipeline();
        if (currPipe == 0) {
            table.getEntry("pipeline").setNumber(1);
        } else {
            table.getEntry("pipeline").setNumber(0);
        }

    }


    /*
     * Gets the horizontal offset from the limelight to the target
     * Outputs in degrees because it sees how far to the left or right the camera is
     */
    public double getHorizontalOffset() {
        
        return horizontalOffset.getDouble(0.0);

    }


    /*
     * Returns the vertical offset angle from the limelight to the target
     */
    public double getVerticalOffset() {

        return verticalOffset.getDouble(0.0);

    }


    /*
     * Returns the skew of the robot
     * Outputs between -90 and 0 degrees
     */
    public double getSkew() {

        return robotSkew.getDouble(0.0);

    }


    /*
     * Returns the total area that the current target takes up on the limelight's screen
     * Outputs a value associated with the percent of the screen being taken up
     */
    public double getTargetArea() {

        return targetArea.getDouble(0.0);

    }


    /*
     * Uses the horizontal offset to determine if the robot is facing left of the target
     * If the horizontal offset is negative it is facing to the left of the target
     */
    public boolean isLookingLeft() {

        return getHorizontalOffset() < 0;

    }


    /*
     * Returns the position of where the robot is on the field
     * This is the form of a number array containing (X, Y, Z) and (Pitch, Yaw, Roll)
     */
    public NetworkTableEntry getRobotPosition3D(){
        
        return robotPosition3D;

    }


    public Rotation2d createRotation2D(){

        /*
         * Gets the pitch value from the robotPositionValues array & converts it to radians
         */
        double robotRotationPitch = robotPositionValues[3].doubleValue();
        double robotRotationPitchRadians = Math.toRadians(robotRotationPitch);

        /*
         * Gets the yaw value from the robotPositionValues array & converts it to radians
         */
        double robotRotationYaw = robotPositionValues[4].doubleValue();
        double robotRotationYawRadians = Math.toRadians(robotRotationYaw);

        /*
         * Uses the pitch & yaw value to construct a rotation2d, then returns it
         */
        Rotation2d limelightRotation2d = new Rotation2d(robotRotationPitchRadians, robotRotationYawRadians);
        return limelightRotation2d;

    }


    public Translation2d createTranslation2D(){
        
        /*
         * Gets the x & y values from the robotPositionValues array
         */
        double robotPositionX = robotPositionValues[0].doubleValue();
        double robotPositionY = robotPositionValues[1].doubleValue();

        /*
         * Uses the x & y values to construct a translation2d, then returns it
         */
        Translation2d limelightTranslation2d = new Translation2d(robotPositionX, robotPositionY);
        return limelightTranslation2d;

    }

    public Transform2d createTransform2D(Translation2d constructorTranslation2d, Rotation2d constructorRotation2d)
    {
        /*
         * Uses a translation2d & a rotation2d parameter to construct a transform2d, then returns it
         */
        Transform2d limelightTransform2d = new Transform2d(constructorTranslation2d, constructorRotation2d);
        return limelightTransform2d;

    }

    
    /*
     * returns a boolean value that lets us know if the limelight has any targets
     */
    public boolean hasTargets()
    {
        return(targets.getBoolean(false));
    }


}