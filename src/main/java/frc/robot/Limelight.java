package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LimelightConstants.*;


public class Limelight implements Sendable{

    public Limelight(){
        SendableRegistry.addLW(this, "Limelight", "Limelight");
    }
    /**
     * Provides an object through which to access the networkTables entries associated with the limelight
     */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Creates a default array of values for use with the botPose table entry
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

    /**
     * Creates a number Array to store the robot position values from the botPose entry
     * Fills it with the numbers from the defaultValues array
     */
    private Number[] getRobotPosition3D()
    {
        Number[] robotPositionValues = table.getEntry("botPose").getNumberArray(defaultValues);
        return robotPositionValues;
    }


     /**
      * Gets the current pipeline of the limelight
      * @return Outputs either 0 or 1: 0 is for RR tape and 1 is for Apriltags
      */
    public int getPipeline() {

        return table.getEntry("pipeline").getNumber(0).intValue();

    }

    /**
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

    /**
     * Gets the horizontal offset angle from networkTable
     * @return The horizontal offset angle from the limelight crosshair to the target
     */
    public Double getHorizontalOffset() {
        
        if(hasTargets()){

        return table.getEntry("tx").getNumber(0).doubleValue();
        
        }
        return Double.NaN;
    }

   

    /**
     * Gets the vertical offset angle from the limelight to the target
     * @return The vertical angle from the limelight crosshair to the target
     */
    public Double getVerticalOffset() {

        if(hasTargets()){

            return 30 + (table.getEntry("ty").getNumber(0).doubleValue());

        }
            return Double.NaN;

    }

   

     /**
      * Outputs between -90 and 0 degrees
      * @return The skew of the target that is currently within the limelight's viewframe
      */
    public Double getSkew() {

        if(hasTargets()){

            return table.getEntry("ts0").getNumber(0).doubleValue();

        }
            return Double.NaN;
        
        
    }

   

     /**
      * Returns the total area that the current target takes up on the limelight's screen
      * Outputs a value associated with the percent of the screen being taken up
      * @return The area of the limelight screen being taken up
      */
    public Double getTargetArea() {

        if(hasTargets()){
        return table.getEntry("ta").getNumber(0).doubleValue();
        }

        return Double.NaN;
    }


     /**
      * Uses the horizontal offset that determines if the robot is looking left
      * @return A boolean that says if the robot is looking left
      */
    public boolean isLookingLeft() {

        return getHorizontalOffset() < 0;

    }

    /**
     * Uses the pitch and yaw values from networkTables to construct and returns a rotation2D
     * @return A rotation2D made from the robot horizontal offset value
     * */
    public Rotation2d createRotation2D(){

        return new Rotation2d(-(Math.toRadians(getHorizontalOffset())));

    }

    /**
     * Uses values from networkTables to construct and return a translation2D
     * @return a translation2D made from the robot x and robot y values, represents movement to the currently seen target
     */
    public Translation2d createTranslation2D() {
        
        /**
         * Gets the x & y values from the robotPositionValues array
         */
        double robotPositionX = getRobotPosition3D()[0].doubleValue();
        double robotPositionY = getRobotPosition3D()[1].doubleValue();

        /**
         * Uses the x & y values to construct a translation2d, then returns it
         */
        return new Translation2d(robotPositionX, robotPositionY);

    }
    /**
     *  Uses an existing translation2d and rotation2d to make a transform2d
     * @param constructorTranslation2d
     * @param constructorRotation2d
     * @return A transform2d (rotates and drives at the same time)
     */
    public Transform2d createTransform2D(Translation2d constructorTranslation2d, Rotation2d constructorRotation2d) {
        /**
         * Uses a translation2d & a rotation2d parameter to construct a transform2d, then returns it
         */
        return new Transform2d(constructorTranslation2d, constructorRotation2d);

    }



     /**
      * returns a boolean value that lets us know if the limelight has any targets
      * @return If the limelight has any targets
      */
    public boolean hasTargets() {
        return (table.getEntry("tv").getDouble(0) > 0);
    }


     /**
      *checks if the limelight is in Apriltag, and if it has a target,  returns the ID of the Apriltag. Otherwise, it returns null.
      * @return The ID of the currently targeted Apriltag
      */
    public Double getTargetID(){
        if(getPipeline() == 1)
        {
            if(hasTargets()){
                return table.getEntry("tid").getDouble(0.0);
            }
         }

        return null;

    }
    
     /**
     * Determines if the limelight is looking at a high-level scoring target
     * @param verticalOffset
     * @return A boolean value of whether the limelight is seeing a high target
     */
         private boolean isHighLevelTarget(double verticalOffset) {
         return verticalOffset > MAX_VERT_OFFSET_FOR_MED && verticalOffset <= MAX_VERT_OFFSET_FOR_HIGH;
        }

    /**
     * Determines if the limelight is looking at a mid-level scoring target
     * @param verticalOffset
     * @return A boolean value of whether the limelight is seeing a middle target
     */
    private boolean isMidLevelTarget(double verticalOffset) {
        return verticalOffset > MAX_VERT_OFFSET_FOR_LOW && verticalOffset <= MAX_VERT_OFFSET_FOR_MED;
    }

    /**
     * Determines if the limelight is looking at a low-level scoring target
     * @param verticalOffset
     * @return A boolean value of whether the limelight sees a human player station target
     */
    private boolean isLowLevelTarget (double verticalOffset) {
        return verticalOffset > 0 && verticalOffset <= MAX_VERT_OFFSET_FOR_LOW;
    }

    /**
     * Determines if the limelight is looking at a Human Player station target
     * @param verticalOffset
     * @return A boolean value of whether the limelight sees a human player station target
     */
    private boolean isHumanPlayerStation (double verticalOffset){
        return verticalOffset > MAX_VERT_OFFSET_FOR_LOW && verticalOffset <= MAX_VERT_OFFSET_FOR_HP_STATION;
    }

    private Pose2d createPose2d(double robotPositionX, double robotPositionY){
        return new Pose2d(robotPositionX, robotPositionY, new Rotation2d(0));
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putString("dfhgjk", "FGHJK");
        builder.setSmartDashboardType("Limelight");
        builder.addBooleanProperty("Has Targets", this::hasTargets, null);
        builder.addDoubleProperty("Horizontal Offset", this::getHorizontalOffset, null);
        builder.addDoubleProperty("Vertical Offset", this::getVerticalOffset, null);
    }
}