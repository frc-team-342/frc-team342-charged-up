package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class Limelight {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry targets = table.getEntry("tv");
    private NetworkTableEntry horizontalOffset = table.getEntry("tx");
    private NetworkTableEntry verticalOffset = table.getEntry("ty");
    private NetworkTableEntry targetArea = table.getEntry("ta");
    private NetworkTableEntry robotPosition3D = table.getEntry("camtran");
    private NetworkTableEntry robotSkew = table.getEntry("ts0");

    private Number[] defaultValues =
    {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };

    private Number[] robotPositionValues = getRobotPosition3D().getNumberArray(defaultValues);




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

    public double getHorizontalOffset() {
        
        return horizontalOffset.getDouble(0.0);

    }

    public double getVerticalOffset() {

        return verticalOffset.getDouble(0.0);

    }

    public double getSkew() {

        return robotSkew.getDouble(0.0);

    }

    public double getTargetArea() {

        return targetArea.getDouble(0.0);

    }

    public boolean isLookingLeft() {

        return getHorizontalOffset() < 0;

    }

    public NetworkTableEntry getRobotPosition3D(){
        
        return robotPosition3D;

    }

    public Rotation2d createRotation2D(){

        double robotRotationPitch = robotPositionValues[3].doubleValue();
        double robotRotationPitchRadians = Math.toRadians(robotRotationPitch);

        double robotRotationYaw = robotPositionValues[4].doubleValue();
        double robotRotationYawRadians = Math.toRadians(robotRotationYaw);

        Rotation2d limelightRotation2d = new Rotation2d(robotRotationPitchRadians, robotRotationYawRadians);
        return limelightRotation2d;

    }

    public Translation2d createTranslation2D(){

        double robotPositionX = robotPositionValues[0].doubleValue();
        double robotPositionY = robotPositionValues[1].doubleValue();

        Translation2d limelightTranslation2d = new Translation2d(robotPositionX, robotPositionY);
        return limelightTranslation2d;

    }

    public Transform2d createTransform2D(Translation2d constructorTranslation2d, Rotation2d constructorRotation2d)
    {

        Transform2d limelightTransform2d = new Transform2d(constructorTranslation2d, constructorRotation2d);
        return limelightTransform2d;

    }

    public boolean hasTargets()
    {
        return(targets.getBoolean(false));
    }

}