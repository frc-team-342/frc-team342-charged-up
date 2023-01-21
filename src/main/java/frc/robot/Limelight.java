package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import static frc.robot.Constants.LimelightConstants.*;


class Limelight {

    /*
     * Creates a network table instance and an entry for each necessary value
     */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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
    public Number[] getRobotPosition3D()
    {
        Number[] robotPositionValues = table.getEntry("camtran").getNumberArray(defaultValues);
        return robotPositionValues;
    }

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
        
        return table.getEntry("tx").getNumber(0).doubleValue();

    }

    /*
     * Returns the vertical offset angle from the limelight to the target
     */
    public double getVerticalOffset() {

        return table.getEntry("tx").getNumber(0).doubleValue();

    }

    /*
     * Returns the skew of the robot
     * Outputs between -90 and 0 degrees
     */
    public double getSkew() {

        return table.getEntry("ts0").getNumber(0).doubleValue();

    }

    /*
     * Returns the total area that the current target takes up on the limelight's screen
     * Outputs a value associated with the percent of the screen being taken up
     */
    public double getTargetArea() {

        return table.getEntry("ta").getNumber(0).doubleValue();

    }

    /*
     * Uses the horizontal offset to determine if the robot is facing left of the target
     * If the horizontal offset is negative it is facing to the left of the target
     */
    public boolean isLookingLeft() {

        return getHorizontalOffset() < 0;

    }

    public Rotation2d createRotation2D() {

        /*
         * Gets the pitch value from the robotPositionValues array & converts it to radians
         */
        double robotRotationPitch = getRobotPosition3D()[3].doubleValue();
        double robotRotationPitchRadians = Math.toRadians(robotRotationPitch);

        /*
         * Gets the yaw value from the robotPositionValues array & converts it to radians
         */
        double robotRotationYaw = getRobotPosition3D()[4].doubleValue();
        double robotRotationYawRadians = Math.toRadians(robotRotationYaw);

        /*
         * Uses the pitch & yaw value to construct a rotation2d, then returns it
         */
        Rotation2d limelightRotation2d = new Rotation2d(robotRotationPitchRadians, robotRotationYawRadians);
        return limelightRotation2d;

    }

    public Translation2d createTranslation2D() {
        
        /*
         * Gets the x & y values from the robotPositionValues array
         */
        double robotPositionX = getRobotPosition3D()[0].doubleValue();
        double robotPositionY = getRobotPosition3D()[1].doubleValue();

        /*
         * Uses the x & y values to construct a translation2d, then returns it
         */
        Translation2d limelightTranslation2d = new Translation2d(robotPositionX, robotPositionY);
        return limelightTranslation2d;

    }

    public Transform2d createTransform2D(Translation2d constructorTranslation2d, Rotation2d constructorRotation2d) {
        /*
         * Uses a translation2d & a rotation2d parameter to construct a transform2d, then returns it
         */
        Transform2d limelightTransform2d = new Transform2d(constructorTranslation2d, constructorRotation2d);
        return limelightTransform2d;

    }

    /*
     * returns a boolean value that lets us know if the limelight has any targets
     */
    public boolean hasTargets() {
        return table.getEntry("tv").getBoolean(false);
    }

    /*
     * Uses the vertical distance from and the angle from the limelight to the low target to calculate the horizontal distance
     * Will need to be updated later as the height and angle are currently unknown
     */
    public double getHorizontalFromLow()
    {
        double verticalOffset = getVerticalOffset();

        if(verticalOffset > 0 && verticalOffset <= Constants.LimelightConstants.MAX_VERT_OFFSET_FOR_LOW)
        {
            double horizontalFromLow = Constants.LimelightConstants.HEIGHT_TO_LOW / Math.tan(verticalOffset);
            return horizontalFromLow;
        }
        
        return 0.0;
    }

    /*
     * Uses the vertical distance from and the angle from the limelight to the mid target to calculate the horizontal distance
     * Will need to be updated later as the height and angle are currently unknown
     */
    public double getHorizontalFromMed()
    {
        double verticalOffset = getVerticalOffset();

        if(verticalOffset > MAX_VERT_OFFSET_FOR_LOW && verticalOffset <= MAX_VERT_OFFSET_FOR_MED)
        {
            double horizontalFromMed = HEIGHT_TO_MED / Math.tan(verticalOffset);
            return horizontalFromMed;
        }

        return 0.0;
    }

    /*
     * Uses the vertical distance from and the angle from the limelight to the high target to calculate the horizontal distance
     * Will need to be updated later as the height and angle are currently unknown
    */
    public double getHorizontalFromHigh()
    {
        double verticalOffset = getVerticalOffset();

        if(verticalOffset > MAX_VERT_OFFSET_FOR_MED && verticalOffset <= MAX_VERT_OFFSET_FOR_HIGH)
        {
            double horizontalFromHigh = HEIGHT_TO_HIGH / Math.tan(verticalOffset);
            return horizontalFromHigh; 
        }

        return 0.0;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Limelight");
        builder.addBooleanProperty("Has Targets", this::hasTargets, null);
        builder.addDoubleProperty("Horizontal Offset", this::getHorizontalOffset, null);
        builder.addDoubleProperty("Vertical Offset", this::getVerticalOffset, null);
        builder.addDoubleProperty("Horizontal Distance from Low Target", this::getHorizontalFromLow, null);
        builder.addDoubleProperty("Horizontal Distance from Med Target", this::getHorizontalFromMed, null);
        builder.addDoubleProperty("Horizontal Distance from High Target", this::getHorizontalFromHigh, null);
    }
}