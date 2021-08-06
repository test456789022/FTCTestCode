package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This class is setup for 2020-2021 game for vuforia vision.
 */
public class Vision {
    //Paste in Vuforia Key. Get from here: https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY =
            "AcE02hn/////AAABmXeRMBKt2EN/tZhAgeZb21lUWnhKa8VoCNaRsqDCShE6T71asG1NBJ4SCPvBywHYNrgK9DdYkcCnY0lcMXU6fH6j0jGVUv4VB2f4a7bAye5FHibhrj2HHiYDoiJfEXN2QjdsVgM3z5pJJ4QQUeiG1tqbyoZpNekfyOfXFdkvIWjr4/K/jP6rOPQGS6TK8YFOAEZfFvr0dPkZTCpF8BsJVuKn7UtXIF7K5Eb1wCy7dxJ0+DfN9PBltugqsilLtDnwr3yi9ZVmLBANsdtRBoRNEvpqEfg8u6hSql5PDh5PPI4I47WObYZCfowes3SOO0H/Vo6dw+MLkUBOsOKi+K/JqtVmfrqRZmS4bKExJy6R2PNo";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private RobotWithVision robot;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float cameraHeight;
    private float cameraLeft;
    private float cameraOut;

    VuforiaTrackables targetsUltimateGoal = null;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    /**
     * The constructor of the Vision() class. Initializes variables to be used later.
     * @param robot The robotWithVision class that has the hardwareMap stored within
     * @param cameraHeight The distance from the floor to the center of the lens in inches
     * @param cameraLeft The distance left of center of the bot that the lens is located in inches.
     *                   Will be negative if the camera is right of center.
     * @param cameraOut The distance out from the center of the bot in inches.
     */
    public Vision(RobotWithVision robot, float cameraHeight, float cameraLeft, float cameraOut) {
        this.robot = robot;
        this.cameraHeight = cameraHeight;
        this.cameraLeft = cameraLeft;
        this.cameraOut = cameraOut;
    }

    public void visionInit() {
        webcamName = robot.hwMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        //int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //We will do a parameterless method since we don't need a preview window
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //Parameters for vuforia. Get your own Vuforia Key
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine with parameter object
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        // This will change Year to Year
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection

        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        //Changes year to year
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.

        // Field coords: https://acmerobotics.github.io/ftc-dashboard/official_field_coord_sys.pdf
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
        // with the wide (horizontal) axis of the camera aligned with the X axis, and
        // the Narrow (vertical) axis of the camera aligned with the Y axis
        //
        // But, this example assumes that the camera is actually facing forward out the front of the robot.
        // So, the "default" camera position requires two rotations to get it oriented correctly.
        // 1) First it must be rotated +90 degrees around the X axis to get it horizontal (it's now facing out the right side of the robot)
        // 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.

        final float CAMERA_FORWARD_DISPLACEMENT  = cameraOut * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = cameraHeight * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = cameraLeft * mmPerInch;     // eg: Camera is 3 inches left of robot-center

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

    }

    //Turn on vision targeting
    public void activateTargeting() {
        targetsUltimateGoal.activate();
    }

    //Turn off vision targeting
    public void deactivateTargeting() {
        targetsUltimateGoal.deactivate();
    }

    //Checks if target is visible, updates vision loop
    public boolean isTargetVisible() {
        targetVisible = updateVision();
        return targetVisible;
    }

    //Loops through all trackable images, if any of them are visible and there has been
    //an update, then update the last location variable. Returns whether or not a target can be seen
    private boolean updateVision() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                return true;
            }
        }
        return false;
    }

    //Returns translation vector
    private VectorF getTranslation() {
        if(targetVisible) {
            return lastLocation.getTranslation();
        }
        return null;
    }

    //Returns orientation of rotation
    private Orientation getRotation() {
        if(targetVisible) {
            return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        }
        return null;
    }

    //Get X component in inches from center field
    public float getX() {
        if(getTranslation() == null) {
            return 0f;
        }
        return getTranslation().get(0) / mmPerInch;
    }

    //Get Y component in inches from center field
    public float getY() {
        if(getTranslation() == null) {
            return 0f;
        }
        return getTranslation().get(1) / mmPerInch;
    }

    //Get Z component (height) in inches from center field (should be nearly constant)
    public float getZ() {
        if(getTranslation() == null) {
            return 0f;
        }
        return getTranslation().get(2) / mmPerInch;
    }

    //Get roll component in degrees from level (should be nearly constant)
    public float getRoll() {
        if(getRotation() == null) {
            return 0f;
        }
        return getRotation().firstAngle;
    }

    //Get pitch component in degrees from level (should be nearly constant)
    public float getPitch() {
        if(getRotation() == null) {
            return 0f;
        }
        return getRotation().secondAngle;
    }

    //Get heading component in degrees from 0
    public float getHeading() {
        if(getRotation() == null) {
            return 0f;
        }
        return getRotation().thirdAngle;
    }

}
