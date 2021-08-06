package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an opmode.
 * This class can be used to define all the specific hardware for a robot
 */
public class RobotWithVision {
    /* Public OpMode members. */

    //Drive
    public DcMotor FRDrive = null;
    public DcMotor FLDrive = null;
    public DcMotor BRDrive = null;
    public DcMotor BLDrive = null;

    public Vision vision = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;


    /* Constructor */
    public RobotWithVision() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FRDrive = hwMap.get(DcMotor.class, "FRDrive");
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        BRDrive = hwMap.get(DcMotor.class, "BRDrive");
        BLDrive = hwMap.get(DcMotor.class, "BLDrive");

        //Most robots need the motor on one side to be reversed to drive forward
        //Reverse the motor that runs backwards when connected directly to the battery
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);

        //Vision constructor. Pass in this robot instance, camera height, and distance from center of robot.
        //See Vision.java for more info about these values.
        vision = new Vision(this, 3f,-0.5f,9.5f);
        vision.visionInit();
    }

    public void DriveMecanum(double strafe, double drive, double turn, boolean modify) {

        //Calculate needed power.
        double modifier = 0.85;
        if (modify) {
            modifier = 0.3;
        }

        double FRPower = -strafe + drive - turn;
        double FLPower = strafe + drive + turn;
        double BRPower = strafe + drive - turn;
        double BLPower = -strafe + drive + turn;

        //Ensure that power does not go over 1.
        double maxPower = Math.max(FLPower, Math.max(FRPower, Math.max(BLPower, BRPower)));

        if (maxPower > 1) {

            FRPower = FRPower / maxPower;
            FLPower = FLPower / maxPower;
            BRPower = BRPower / maxPower;
            BLPower = BLPower / maxPower;

        }

        //Apply the power to the wheels.
        FRDrive.setPower(FRPower * modifier);
        FLDrive.setPower(FLPower * modifier);
        BRDrive.setPower(BRPower * modifier);
        BLDrive.setPower(BLPower * modifier);
    }


    //Move to a specific position on the field with a given rotation
    public void moveToPos(float x, float y, float rot, float tolerance, float rotTolerance) {
        //If everything is within tolerance, then return without moving
        if(withinTolerance(x, vision.getX(), tolerance) && withinTolerance(y, vision.getY(), tolerance) && withinTolerance(vision.getHeading(),rot,rotTolerance)) {
            return;
        }
        float turn = 0;
        //If the rotation is not within the tolerance, turn a small amount towards the direction you are trying to go.
        //You want a small turn amount due to how the math behind DriveMecanum works
        if(!withinTolerance(vision.getHeading(), rot, rotTolerance)) {
            float diff = vision.getHeading()-rot;
            if(diff > 0) {
                turn = 0.2f;
            } else {
                turn = -0.2f;
            }
        }
        /*
         * High Level: We need to get the direction we want the robot to move in so we can get to the location.
         * In order to do so, we take the location we want (in x and y for horizontal and vertical) and
         * subtract where we are now (x and y from vision). This gives us how much we need to move vertically and
         * horizontally.
         * Using this information we can get a direction using the Math.atan2() function.
         * This is a tangent function that will give us the direction we are facing using x and y values.
         * Then we need to figure out which direction we need the robot to move in based on what direction it is facing now.
         * We can simply subtract where we are looking by our desired heading to get the direction the robot needs to move in.
         * From there we can use the heading to get the x power and y power by using cosine and sine functions.
         */
        /*
         * Note: Math library is in radians, so any degree values need to be converted
         * MODIFIER - The max power for the strafe and drive components so it doesn't go too fast
         * DeltaX - Difference between desired and current x location
         * DeltaY - Difference between desired and current y location
         * Dist - Distance to location (pythagorean theorem is fun)
         * If distance is smaller than 10 inches, then we can decrease max power so it slows down towards the desired location
         * Heading - Direction from current location to desired location
         * DeltaHeading - Direction to go from the front of the robot
         * Drive - Power given to the drive component. Cosine of the heading times the current multiplier.
         * Strafe - Power given to the strafe component. Sine of the heading times the current multiplier.
         * Send all this to the DriveMecanum method to drive to location
         */
        //Rectangular to polar, get the difference between current heading and desired heading,
        //then adapt the angle to a unit circle to get the driving power
        float MODIFIER = 0.5f;
        float deltaX = x - vision.getX();
        float deltaY = y - vision.getY();
        float dist = (float) Math.sqrt(deltaX*deltaX+deltaY*deltaY);
        if(dist < 10) {
            MODIFIER *= (dist / 10);
        }
        float heading = (float) Math.atan2(deltaY, deltaX);
        float deltaHeading = (float)Math.toRadians(vision.getHeading()) - heading;
        //Drive is X coordinate, Strafe is Y coordinate
        float drive = (float)Math.cos(deltaHeading)*MODIFIER;
        float strafe = (float)Math.sin(deltaHeading)*MODIFIER;
        DriveMecanum(strafe, drive, turn, false);
    }

    //Helper method to do quick tolerance checks
    public static boolean withinTolerance(float value1 ,float value2, float tolerance) {
        return Math.abs(value1-value2) < tolerance && Math.abs(value2-value1) < tolerance;
    }
}







