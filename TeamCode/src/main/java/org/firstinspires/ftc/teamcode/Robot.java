package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This is NOT an opmode.
 * This class can be used to define all the specific hardware for a robot
 */
public class Robot {
    /* Public OpMode members. */

    //Drive
    public DcMotor FRDrive = null;
    public DcMotor FLDrive = null;
    public DcMotor BRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor Pickup1 = null;
    public DcMotor Pickup2 = null;
    public DcMotor Shooter = null;
    public DcMotor Arm = null;

    public Servo Gripper = null;
    public Servo Color = null;
    public Servo Kicker = null;

    public ColorSensor colorSensor1 = null;
    public ColorSensor colorSensor2 = null;
   // public ColorSensor colorSensor3 = null;
   // public ColorSensor colorSensor4 = null;

    public TouchSensor touchSensor1 = null;
    public TouchSensor touchSensor2 = null;

    public int ringRH = 3600;
    public int ringRL = 120;
    public int ringGH = 3165;
    public int ringGL = 165;
    public int ringBH = 900;
    public int ringBL = 115;
    public int ringAlphaH = 2575;
    public int ringAlphaL = 135;


    /*public int ringR = 600;
    public int ringG = 600;
    public int ringB = 300;
    public int ringAlpha = 500;
    public int offset = 100;*/


    /* local OpMode members. */
    HardwareMap hwMap = null;
    // (https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html) Goto robotcore.util -> ElapsedTime
    // Edited to have the timer on the millisecond time interval
    private ElapsedTime period = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /* Enum (https://www.w3schools.com/java/java_enums.asp) 
       Holds the different state values that the program can be in
       Below is an example, can add/remove values at will
       Usage: State.MANUAL
    */
    public enum State {
        MANUAL,
        INIT,
        LEFT,
        STRAIGHT,
        SHOOT,
        EXIT
    }

    //Holds current state that the robot is in. Begin in manual mode
    public State currentState = State.MANUAL;


    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //Drive
        FRDrive = hwMap.get(DcMotor.class, "FRDrive");
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        BRDrive = hwMap.get(DcMotor.class, "BRDrive");
        BLDrive = hwMap.get(DcMotor.class, "BLDrive");
        Pickup1 = hwMap.get(DcMotor.class, "Pickup1");
        Pickup2 = hwMap.get(DcMotor.class, "Pickup2");
        Shooter = hwMap.get(DcMotor.class, "Shooter");
        Arm = hwMap.get(DcMotor.class, "Arm");

        Gripper = hwMap.get(Servo.class, "Gripper");
        Color = hwMap.get(Servo.class, "Color");
        Kicker = hwMap.get(Servo.class, "Kicker");

        colorSensor1 = hwMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");
        //colorSensor3 = hwMap.get(ColorSensor.class, "colorSensor3");
        //colorSensor4 = hwMap.get(ColorSensor.class, "colorSensor4");

        touchSensor1 = hwMap.get(TouchSensor.class, "touchSensor1");
        touchSensor2 = hwMap.get(TouchSensor.class, "touchSensor2");

        //Most robots need the motor on one side to be reversed to drive forward
        //Reverse the motor that runs backwards when connected directly to the battery
        //Drive
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);

        Pickup1.setDirection(DcMotor.Direction.FORWARD);
        Pickup2.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        //Drive
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);

        Pickup1.setPower(0);
        Pickup2.setPower(0);
        Shooter.setPower(0);
        Arm.setPower(0);


        // Set all motors to run without encoders.

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Pickup1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pickup2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveMecanum(double strafe, double drive, double turn, boolean modify) {

        //Calculate needed power.
        double modifier = 0.85;
        if (modify) {
            modifier = 0.3;
        } else {
            modifier = 0.85;
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

    //Motors

    public void Pickup(boolean in, boolean out) {

        if (in && !out) {
            Pickup1.setPower(1);
            Pickup2.setPower(1);

        } else if (out && !in) {
            Pickup1.setPower(-1);
            Pickup2.setPower(-1);

        } else
            Pickup1.setPower(0);
            Pickup2.setPower(0);


    }
    public void Shooter(boolean in, boolean out) {
        if (in && !out) {
            Shooter.setPower(0.70);

        } else if (out && !in) {
            Shooter.setPower(0.60);

        }else
            Shooter.setPower(0);
    }

    public void Arm(boolean down, boolean up) {

        if (up && !down && !touchSensor2.isPressed()) {
            Arm.setPower(0.4);

        } else if (down && !up && !touchSensor1.isPressed()) {
            Arm.setPower(-0.4);

        } else {
            Arm.setPower(0);
        }
    }


    // Define and initialize ALL installed servos.
    public void Gripper(boolean open, boolean close) {
        if (open && !close) {
            Gripper.setPosition(0.6);

        }
        if (close && !open) {
            Gripper.setPosition(1);

        }
    }

    public void Color(boolean up, boolean down) {
        if (up && !down) {
            Color.setPosition(0.58);

        }
        if (down && !up) {
            Color.setPosition(0.15);


        }
    }

    public void Kicker(boolean back, boolean forward) {

        if (back && !forward) {
            Kicker.setPosition(0.65);

        } else if (forward && !back) {
            Kicker.setPosition(0.40);

        } else {
            Kicker.setPosition(0.65);


        }
    }

    public boolean ringPresent(ColorSensor colorSensor) {
        boolean isPresent;
        if (colorSensor.red() < ringRH && colorSensor.red() > ringRL) {
            if (colorSensor.green() < ringGH || colorSensor.green() > ringGL) {
                if (colorSensor.blue() < ringBH || colorSensor.blue() > ringBL) {
                    if (colorSensor.alpha() < ringAlphaH && colorSensor.alpha() > ringAlphaL) {
                        isPresent = true;
                    } else {
                        isPresent = false;
                    }
                } else {
                    isPresent = false;
                }
            } else {
                isPresent = false;
            }
        } else {
            isPresent = false;
        }

        return isPresent;
    }

    /* We will set up a state machine for semi-automatic robot moves
       Based on the value of currentState, the robot will be able to do different things
       Each state will have its own independent function that is called when that state needs to be called
       Each state needs an exit condition (Except MANUAL) That will bring us to the next state once a condition is met
        or else we are stuck in an infitite loop
       We will add an e-stop that puts the state back in MANUAL for the drivers just in case something breaks
    */

    public void checkForStateInput(boolean stop, boolean shoot) {
        //Checks if the e-stop was initiated, if so then immediately put the robot back into MANUAL mode
        if(stop) {
            currentState = State.MANUAL;
            return;
        }
        //If we are currently in the manual mode and the shoot button is activated, then set the state machine in motion
        if(currentState == State.MANUAL && shoot) {
            currentState = State.INIT;
            return;
        }
    }

    /* Runs the routine for the shoot state machine
       List of what this shoot state machine does:
       Turn left for 2 seconds
       Go straight for 1 second
       Shoot for 2 seconds
    */
    public void shootStateMachine() {
        switch(currentState) {
            //Reset the period timer, lets us time everything. Zero everything so any manual instructions do not affect semi-auto code
            case INIT:
                period.reset();
                currentState = State.LEFT;
                DriveMecanum(0,0,0,false);
                Pickup(false, false);
                Shooter(false, false);
                Arm(false, false);
                Gripper(false, false);
                Kicker(false, false);
                Color(false, false);
                break;
            //Use the period timer to check if we need to go to next state, else turn left
            case LEFT:
                if(period.seconds() >= 2) {
                    currentState = State.STRAIGHT;
                    period.reset();
                } else {
                    DriveMecanum(-0.5,0,0,false);
                }
                break;
            //Use the period timer to check if we need to go to next state, else go straight
            case STRAIGHT:
                if(period.seconds() >= 1) {
                    DriveMecanum(0,0,0,false); //Need this here to stop the robot, else you will keep driving straight or error out
                    currentState = State.SHOOT;
                    period.reset();
                } else {
                    DriveMecanum(0,0.5,0,false);
                }
                break;
            //Use timer to tell if we need to run the kicker at certain times, keep the shooter running whole time
            case SHOOT:
                Shooter(false, true);
                if(period.milliseconds() > 50 && period.milliseconds() < 200) {
                    Kicker(false, true);
                } else if (period.milliseconds() >= 200 && period.milliseconds() < 1000) {
                    Kicker(true, false);
                } else if (period.seconds() >= 1) {
                    Kicker(false, false);
                    Shooter(false, false);
                    currentState = State.EXIT;
                }
                break;
            //Cleanup anything here if needed
            case EXIT:
                currentState = State.MANUAL;
                break;
        }
    }





}

    /*public boolean ringPresent(ColorSensor colorSensor){
        boolean isPresent;
        if(colorSensor.red() < ringR+offset && colorSensor.red() > ringR-offset) {
            if (colorSensor.green() < ringG + offset && colorSensor.green() > ringG - offset) {
                if (colorSensor.blue() < ringB + offset && colorSensor.blue() > ringB - offset) {
                    isPresent = colorSensor.alpha() < ringAlpha + offset && colorSensor.alpha() > ringAlpha - offset;
                }else{
                    isPresent = false;
                }
            }else{
                isPresent = false;
            }
        }else{
            isPresent = false;
        }

        return isPresent;
    }*/



