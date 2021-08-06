package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class Teleop extends OpMode {
    // Declare OpMode members.

    Robot robot = new Robot();

    private ElapsedTime runtime = new ElapsedTime();
    double FRpower;
    double FLpower;
    double BRpower;
    double BLpower;

    boolean running = false;
    int step = 0;
    double tempPos = 0;
    double tempX = 0;
    double tempY = 0;
    double tempZ = 0;

    //Set up variables.
    static final double COUNTS_PER_MOTOR_REV = 383.6;  //GoBilda Motor 28 counts per motor rev (28*13.7=383.6)
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.93734;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double LOW_SPEED = 0.30;
    static final double HIGH_SPEED = 0.45;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters

        robot.init(hardwareMap);

        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
       robot.Color.setPosition(0.58);
        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Arm.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shooter.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.checkForStateInput(gamepad1.a, gamepad1.y);

        if(robot.currentState == Robot.State.MANUAL) {
            robot.DriveMecanum(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0);
            robot.Pickup(gamepad2.y, gamepad2.a);
            robot.Shooter(gamepad2.right_trigger > 0, gamepad2.left_trigger> 0);
            robot.Arm(gamepad2.dpad_up, gamepad2.dpad_down);

            robot.Gripper(gamepad2.dpad_right, gamepad2.dpad_left);
            robot.Kicker(gamepad2.right_bumper, gamepad2.left_bumper);
            robot.Color (gamepad2.b, gamepad2.x);
        } else {
            robot.shootStateMachine();
        }

       

        telemetry.addData("Red Bottom", robot.colorSensor1.red());
        telemetry.addData("Blue Bottom", robot.colorSensor1.blue());
        telemetry.addData("Green Bottom", robot.colorSensor1.green());
        telemetry.addData("Alpha Bottom", robot.colorSensor1.alpha());

        telemetry.addData("Red Top", robot.colorSensor2.red());
        telemetry.addData("Blue Top", robot.colorSensor2.blue());
        telemetry.addData("Green Top", robot.colorSensor2.green());
        telemetry.addData("Alpha Top", robot.colorSensor2.alpha());

       /* telemetry.addData("Middle Red Top", robot.colorSensor3.red());
        telemetry.addData("Middle Blue Top", robot.colorSensor3.blue());
        telemetry.addData("Middle Green Top", robot.colorSensor3.green());
        telemetry.addData("Middle Alpha Top", robot.colorSensor3.alpha());

        telemetry.addData("Middle Red Bottom", robot.colorSensor4.red());
        telemetry.addData("Middle Blue Bottom", robot.colorSensor4.blue());
        telemetry.addData("Middle Green Bottom", robot.colorSensor4.green());
        telemetry.addData("Middle Alpha Bottom", robot.colorSensor4.alpha());
*/
        telemetry.addData("Is ring in front of bottom sensor:" , robot.ringPresent(robot.colorSensor1));
        telemetry.addData("Is ring in front of top sensor:" , robot.ringPresent(robot.colorSensor2));
        //telemetry.addData("Is ring in front of middle top sensor:" , robot.ringPresent(robot.colorSensor3));
        //telemetry.addData("Is ring in front of middle bottom sensor:" , robot.ringPresent(robot.colorSensor4));

        telemetry.addData("FRDrive", robot.FRDrive.getCurrentPosition());
        telemetry.addData("FLDrive", robot.FLDrive.getCurrentPosition());
        telemetry.addData("BRDrive", robot.BRDrive.getCurrentPosition());
        telemetry.addData("BLDrive", robot.BLDrive.getCurrentPosition());
        telemetry.addData("Pickup", robot.Pickup1.getCurrentPosition());
        telemetry.addData("Shooter", robot.Shooter.getCurrentPosition());
        telemetry.addData("Arm", robot.Arm.getCurrentPosition());



        // Show the elapsed game time and wheel power.,
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "FRpower (%.2f), FLpower (%.2f), BRpower (%.2f), BLpower (%.2f)", FRpower, FLpower, BRpower, BLpower);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
