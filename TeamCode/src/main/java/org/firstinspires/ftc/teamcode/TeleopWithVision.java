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

@TeleOp(name="TeleOpWithVision", group="Iterative Opmode")
//@Disabled
public class TeleopWithVision extends OpMode {
    // Declare OpMode members.

    RobotWithVision robot = new RobotWithVision();

    private ElapsedTime runtime = new ElapsedTime();
    double FRpower;
    double FLpower;
    double BRpower;
    double BLpower;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

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

        robot.FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Activate vision targeting
        robot.vision.activateTargeting();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        //Get info if target is visible and loops vision code
        boolean targetVisible = robot.vision.isTargetVisible();

        //If there is no visible target, revert to manual driving
        //There are examples of the move to position values being used
        //If nothing is selected, manual driving should be used
        if(!targetVisible) {
            robot.DriveMecanum(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0);
        } else if(gamepad2.a) {
            robot.moveToPos(30,-30,0,1, 5);
        } else if(gamepad2.b) {
            robot.moveToPos(30,30,0,1, 5);
        } else if(gamepad2.x) {
            robot.moveToPos(0,-30,-90,1, 10);
        } else if(gamepad2.y) {
            robot.moveToPos(0,30,90,1, 10);
        } else {
            robot.DriveMecanum(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0);
        }


        //Send telemetry data to the driver station
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "FRpower (%.2f), FLpower (%.2f), BRpower (%.2f), BLpower (%.2f)", FRpower, FLpower, BRpower, BLpower);
        telemetry.addData("Encoders", "FRpos (%d), FLpos (%d), BRpos (%d), BLpos (%d)",robot.FRDrive.getCurrentPosition(),robot.FLDrive.getCurrentPosition(), robot.BRDrive.getCurrentPosition(), robot.BLDrive.getCurrentPosition());
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", robot.vision.getX(), robot.vision.getY(), robot.vision.getZ());
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robot.vision.getRoll(), robot.vision.getPitch(), robot.vision.getHeading());
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //Make sure to deactivate targeting system so resources aren't wasted
        robot.vision.deactivateTargeting();
    }

}
