package org.firstinspires.ftc.teamcode.RidgeviewRoboticsFTCSkystone13022;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RidgeviewRoboticsFTCSkystone13022.HardwareSkystone;

import java.util.List;

@Autonomous(name = "SkystoneAutonomousBlue", group = "Linear Opmode")
public class SkystoneAutonomousBlue extends LinearOpMode
{

    // Global Variables


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXX6Dh7/////AAABmbgZiIyG1E9ThQ6TXfPkd8M6i6PCEXDZjkt2SpXyFdQmm+CuRQp" +
                    "6A9osXLHPk1eusykK9vUsTv4XVG+T9Ikgor/aU0YALMCsVj6t0igpi5T31wEak6" +
                    "wMpvaJTSS1uEJ99G99yLh6TsHIMHBRICuGERoSBfnRwowu2/FYIv8eFsN52R++XyF" +
                    "nC8DGk/XIZSeWb6v5adayCKLUaUSBXeuTAZLHj17j0TJojO3osifAfe2JmHwKkFYozZ" +
                    "kkzt+eRkImzOFPkcnE3D2otxVbbPNqUtns0lo0Dh7Ifd0p+M4MfWeWXHf/kN19Q7pIHqf3ie" +
                    "nh58D/8Y+zsoSHtoc/62SgM+K+oVhscYHu/DpIVAFo0IXA";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    private static final double P_TURN_COEFF = 8.0;
    private static final double I_TURN_COEFF = 0.0;
    private static final double D_TURN_COEFF = 4.5;


    private static final double P_DRIVE_COEFF = 5.0;
    private static final double I_DRIVE_COEFF = 0.0;
    private static final double D_DRIVE_COEFF = 0.15;

    private static final double P_LIFT_COEFF = 0.07;
    private static final double I_LIFT_COEFF = 0.07;
    private static final double D_LIFT_COEFF = 0.07;

    private static final double HEADING_THRESHOLD = 0.5;
    private static final double MIN_TURN_SPEED = 0.3;
    private static final double TURN_SPEED = 0.8;


    double MOVE_ENCODER_TOLERANCE = 10;

    // Declare Timers
    private ElapsedTime runtime = new ElapsedTime();


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;

    // Servo Positions
    final double ROTATE_END_MIDDLE = 0.57;
    final double ROTATE_END_BACK = 0.85;
    final double ROTATE_END_FORWARD = 0.1;

    final double ROTATE_RIGHT = 0.84;
    final double ROTATE_LEFT = 0.26;

    final double PINCH_DOWN = 1.0;
    final double PINCH_UP = 0.5;

    final double BLOCK_HEIGHT = 1.5;

    final double CLAMP_RELEASED = 0.95;
    final double CLAMP_ENGAGED = 0.67;

    final double AUTONOMOUS_DOWN = 0.53;
    final double AUTONOMOUS_UP = 0.15;

    double hook1UpPos = 0.9;
    double hook1DownPos = 0.1;

    double hook2UpPos = 0.1;
    double hook2DownPos = 0.8;

    // Declare Variables
    double[] dataX = new double[20];
    double[] dataY = new double[20];
    int level = 1;
    boolean armPositionState;    //true == Home
    double currentAngle;
    boolean prevIntakeState = false;
    boolean is_engaged = false;

    HardwareSkystone hardware = new HardwareSkystone();

    PID_Controller leftBackPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
    PID_Controller leftFrontPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
    PID_Controller rightBackPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
    PID_Controller rightFrontPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);

    PID_Controller gyroPID = new PID_Controller(P_TURN_COEFF, I_TURN_COEFF, D_TURN_COEFF);

    PID_Controller liftPID = new PID_Controller(P_LIFT_COEFF, I_LIFT_COEFF, D_LIFT_COEFF);

    String cubePos = "Left";

    @Override
    public void runOpMode()
    {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        sleep(100);

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        } else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null)
        {
            tfod.activate();
        }

        telemetry.addData("TFOD: ", "Initialized");
        telemetry.update();

        //Initialization Period
        hardware.init(hardwareMap);

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        // Initalizes The Arm Position
        armHome();
        clampRelease();

        sleep(100);


        hardware.autonomousServo.setPosition(AUTONOMOUS_UP);


        telemetry.addData(">:", "Ready For Start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)//////////////////
        /////////////////////////////////////////////////////////////////////
        waitForStart();

        driveEncoderPID(23);

        gyroTurnPID(0);

        while (opModeIsActive())
        {
            if (tfod != null)
            {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null)
                {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.

                    //int i = 0;
                   /* for (Recognition recognition : updatedRecognitions)
                    {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
*/

                    telemetry.addData("NUM OF CUBES: ", updatedRecognitions.size());
                    telemetry.update();


                    if (updatedRecognitions.size() >= 2)
                    {
                        double object1Pos = updatedRecognitions.get(0).getLeft();
                        double object2Pos = updatedRecognitions.get(1).getLeft();

                        String object1 = updatedRecognitions.get(0).getLabel();
                        String object2 = updatedRecognitions.get(1).getLabel();

                        //debugging
                       /* telemetry.addData("IN ", "IF");
                        telemetry.addData("NUM OF CUBES: ", updatedRecognitions.size());

                        telemetry.addData("Object1 :", object1);
                        telemetry.addData("Object1 Pos: ", object1Pos);
                        telemetry.addData("Object2 : ", object2);
                        telemetry.addData("Object2 Pos: ", object2Pos);

                        telemetry.update();
                        sleep(2000);


                        */

                        if (object1.equals("Stone") && object2.equals("Stone"))
                        {
                            cubePos = "Right";
                            break;
                        }


                        if (object1Pos > object2Pos)
                        {
                            if (object1.equals("Skystone"))
                            {
                                cubePos = "Middle";
                                break;
                            }
                        }
                        else if (object1Pos < object2Pos)
                        {
                            if (object1.equals("Skystone"))
                            {
                                cubePos = "Left";
                                break;
                            }
                        }
                        else
                        {
                            cubePos = "Right";
                            break;
                        }


                    }

                }
            }
        }

        tfod.shutdown();


        if (cubePos.equals("Left"))
        {
            // Left Case
            telemetry.addData("Cube Posistion:", "LEFT");
        } else if (cubePos.equals("Middle"))
        {
            // Middle Case
            telemetry.addData("Cube Posistion:", "MIDDLE");
        } else
        {
            // Right Case
            telemetry.addData("Cube Posistion:", "RIGHT");
            driveEncoderPID(5);
            gyroTurnPID(-90);
            strafeForTime(0.2, 0.9, true);
            driveEncoderPID(6);
            autonomousServoDown();
            sleep(500);
            strafeForTime(0.3, 1.5, false);
            gyroTurnPID(-90);
            driveEncoderPID(-60);
            gyroTurnPID(-90);
            autonomousServoUp();
            sleep(500);
            driveEncoderPID(20);
        }


        telemetry.update();


        sleep(1000);
    }

    /////////// All Methods Below////////////

    void mecanumControl()
    {
        double drive;   // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotate;  // Power for rotating the robot


        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower = drive - strafe + rotate;
        double backLeftPower = drive + strafe + rotate;
        double frontRightPower = drive + strafe - rotate;
        double backRightPower = drive - strafe - rotate;

        hardware.leftFront.setPower(frontLeftPower);
        hardware.rightFront.setPower(frontRightPower);
        hardware.leftBack.setPower(backLeftPower);
        hardware.rightBack.setPower(backRightPower);
    }

    void intakeControl()
    {

        if (gamepad1.left_trigger < gamepad1.right_trigger)
        {
            hardware.rightIntake.setPower(-gamepad1.right_trigger);
            hardware.leftIntake.setPower(-gamepad1.right_trigger);
        } else
        {
            hardware.rightIntake.setPower(gamepad1.left_trigger);
            hardware.leftIntake.setPower(gamepad1.left_trigger);
        }

    }

   /* void hookControl ()
    {
        if (gamepad1.a)
        {
            if (!aValue)
            {
                hardware.hook1.setPosition(hook1DownPos);
                hardware.hook2.setPosition(hook2DownPos);
                xBool = true;
                telemetry.addData(">", "Hook Down");
            }
            if (aValue)
            {
                hardware.hook1.setPosition(hook1UpPos);
                hardware.hook2.setPosition(hook2UpPos);
                xBool = false;
                telemetry.addData(">", "Hook Up");
            }
            aValue = xBool;
            telemetry.update();

        }
    } */

    // Slows a Servo's speed down to a slower constant speed
    void servoSlow(Servo servo, double pos)
    {
        double moveRate = 0.01;
        int delayRate = 10;
        if (servo.getPosition() > pos)   //Backwards Rotation
        {
            while (servo.getPosition() > pos)
            {
                servo.setPosition(servo.getPosition() - moveRate);
                sleep(delayRate);
            }
            servo.setPosition(pos);
        } else  // Forwards Rotation
        {
            while (servo.getPosition() < pos)
            {
                servo.setPosition(servo.getPosition() + moveRate);
                sleep(delayRate);
            }
            servo.setPosition(pos);
        }

    }

    void autoStack()
    {
        if (level <= 9 && level > 1)
        {

            //Pinch Block
            pinchDown();
            sleep(200);
            clampRelease();
            sleep(400);


            //Gets Ready to Drop Block
            encoderMove(0.7, ((BLOCK_HEIGHT * (level - 1)) + 1), 15, hardware.liftMotor);
            armOut();
            sleep(1500);

            //Sets the Block Down a Little
            encoderMove(0.3, -1.5, 15, hardware.liftMotor);
            sleep(100);

            //Drops Block
            pinchUp();
            sleep(250);

            //Resets Lift System
            encoderMove(0.3, 1.5, 15, hardware.liftMotor);
            sleep(100);
            armHome();
            sleep(100);
            encoderMove(0.7, -((BLOCK_HEIGHT * (level - 1)) + 1), 15, hardware.liftMotor);
            level++;
        }

        if (level == 1)
        {
            //Pinch Block
            pinchDown();
            sleep(200);
            clampRelease();
            sleep(400);


            //Gets Ready to Drop Block
            encoderMove(0.7, 1.5, 15, hardware.liftMotor);
            armOut();
            sleep(1500);

            //Sets the Block Down a Little
            encoderMove(0.3, -2.3, 15, hardware.liftMotor);
            sleep(100);

            //Drops Block
            pinchUp();
            sleep(250);

            //Resets Lift System
            encoderMove(0.3, 2.3, 15, hardware.liftMotor);
            sleep(100);
            armHome();
            sleep(100);
            encoderMove(0.7, -1.5, 15, hardware.liftMotor);
            level++;
        }
    }

    void autoStack(boolean command)
    {
        if (command)
        {
            autoStack();
        }
    }

    void pinchServoControl(boolean command)
    {
        if (command)
        {
            if (hardware.pinchServo.getPosition() == PINCH_DOWN)
            {
                pinchUp();
            } else
            {
                pinchDown();
            }
        }
    }

    void rotateEndServoControl(boolean command)
    {
        if (command)
        {
            if (hardware.rotateEndServo.getPosition() == ROTATE_END_BACK)
            {
                hardware.rotateEndServo.setPosition(ROTATE_END_FORWARD);
            } else
            {
                hardware.rotateEndServo.setPosition(ROTATE_END_BACK);
            }
        }
    }

    void liftSystemManualControl(boolean upCommand, boolean downCommand)
    {
        if (upCommand)
        {
            hardware.liftMotor.setPower(0.1);
        } else
        {
            hardware.liftMotor.setPower(0);
        }
        if (downCommand)
        {
            hardware.liftMotor.setPower(0.1);
        } else
        {
            hardware.liftMotor.setPower(0);
        }
    }

    void armControl(boolean command)
    {

        if (command)
        {
            if (armPositionState)
            {
                armOut();
            } else
            {
                armHome();
            }
        }
    }

    void armHome()
    {
        //get ready to collect another cube
        pinchUp();
        //rotate left
        hardware.rotateServo.setPosition(ROTATE_RIGHT);
        //towards back
        hardware.rotateEndServo.setPosition(ROTATE_END_FORWARD);
    }

    void clampRelease()
    {
        hardware.clampServo.setPosition(CLAMP_RELEASED);
    }

    void clampEngage()
    {
        hardware.clampServo.setPosition(CLAMP_ENGAGED);
    }

    void autonomousServoDown()
    {
        hardware.autonomousServo.setPosition(AUTONOMOUS_DOWN);
    }

    void autonomousServoUp()
    {
        hardware.autonomousServo.setPosition(AUTONOMOUS_UP);
    }

    void clampControl(boolean command)
    {

        if (command)
        {
            clampEngage();
        } else if (gamepad1.b)
        {
            clampRelease();
        }
    }

    void armOut()
    {
        pinchDown();
        servoSlow(hardware.rotateServo, ROTATE_LEFT);
        servoSlow(hardware.rotateEndServo, ROTATE_END_MIDDLE);
    }

    void pinchUp()
    {
        //Drops Block
        hardware.pinchServo.setPosition(PINCH_UP);
    }

    void pinchDown()
    {
        //Pinch Block
        hardware.pinchServo.setPosition(PINCH_DOWN);
    }

    // neg speed goes back
    void straight(double speed)
    {
        speed = -speed;
        hardware.leftFront.setPower(speed);
        hardware.rightFront.setPower(speed);
        hardware.leftBack.setPower(speed);
        hardware.rightBack.setPower(speed);
    }

    // neg speed goes right
    void strafe(double speed)
    {
        hardware.leftFront.setPower(-speed);
        hardware.rightFront.setPower(speed);
        hardware.leftBack.setPower(speed);
        hardware.rightBack.setPower(-speed);
    }

    // neg speed turns right
    void turn(double speed)
    {
        hardware.leftFront.setPower(-speed);
        hardware.rightFront.setPower(speed);
        hardware.leftBack.setPower(-speed);
        hardware.rightBack.setPower(speed);
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed, double distance, double angle)
    {
        final double CLICKS_PER_REV = 383.6;    // Motor Encoder ticks for GoBilda 435RPM
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double CLICKS_PER_INCH = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {


            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * CLICKS_PER_INCH);
            newLeftFrontTarget = hardware.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = hardware.rightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = hardware.leftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = hardware.rightBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            hardware.leftFront.setTargetPosition(newLeftFrontTarget);
            hardware.rightFront.setTargetPosition(newRightFrontTarget);
            hardware.leftBack.setTargetPosition(newLeftBackTarget);
            hardware.rightBack.setTargetPosition(newRightBackTarget);

            hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            hardware.leftFront.setPower(speed);
            hardware.rightFront.setPower(speed);
            hardware.leftBack.setPower(speed);
            hardware.rightBack.setPower(speed);

            // Adjust angle to be positive 0 - 360
            if (angle < 0)
                angle += 360;

            // keep looping and adjusting steer angle while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (hardware.leftFront.isBusy() && hardware.rightFront.isBusy() && hardware.leftBack.isBusy() && hardware.rightBack.isBusy()))
            {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                Range.clip(steer, -TURN_SPEED, TURN_SPEED);
                leftSpeed = steer;
                rightSpeed = -steer;

                hardware.leftFront.setPower(leftSpeed);
                hardware.rightFront.setPower(rightSpeed);
                hardware.leftBack.setPower(leftSpeed);
                hardware.rightBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Actual", "%7d:%7d", hardware.leftFront.getCurrentPosition(),
                        hardware.rightFront.getCurrentPosition());
                telemetry.addData("Steer Speed", "%5.2f, %5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            hardware.leftFront.setPower(0);
            hardware.rightFront.setPower(0);
            hardware.leftBack.setPower(0);
            hardware.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double angle, double maxTimeMS)
    {

        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        if (angle < 0)
            angle += 360;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(angle, P_TURN_COEFF) && (turnTimer.milliseconds() < maxTimeMS))
        {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.addData(">:", "Gyro Turn is Looping");
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double angle, double holdTime)
    {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime))
        {
            // Update telemetry & Allow time for other processes to run.
            onHeading(angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double angle, double PCoeff)
    {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            steer = 0.0;
            onTarget = true;
        } else
        {
            steer = getSteer(error, PCoeff);
            steer = gyroPID.getSpeed(error);
        }

        // Send desired speeds to motors.
        leftSpeed = steer;
        rightSpeed = -steer;
        hardware.leftFront.setPower(leftSpeed);
        hardware.rightBack.setPower(rightSpeed);
        hardware.leftBack.setPower(leftSpeed);
        hardware.rightFront.setPower(rightSpeed);

        // Display it for the driver.
        //telemetry.addData("Target", "%5.2f", angle);
        //telemetry.addData("Err/St", "%5.2f / %5.2f", error, steer);
        // telemetry.addData("Speed Steer.", "%5.2f, %5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle)
    {

        double robotError;
        double currentAngle;
        double clockwiseAngle;
        double counterClockwiseAngle;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        //while (robotError > 180)  robotError -= 360;
        //while (robotError <= -180) robotError += 360;

        //return robotError;

        // Get current angle and adjust to be in range of 0 - 360 (CCW positive) IMU gives angle in +/- 180 angle from original angle
        currentAngle = hardware.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        if (currentAngle < 0)
            currentAngle += 360;

        clockwiseAngle = currentAngle - targetAngle;
        if (clockwiseAngle < 0)
            clockwiseAngle += 360;

        counterClockwiseAngle = targetAngle - currentAngle;
        if (counterClockwiseAngle < 0)
            counterClockwiseAngle += 360;


        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
        //sleep(100);

        if (counterClockwiseAngle <= clockwiseAngle)
            return counterClockwiseAngle;               // Turn Left (positive)
        else
            return -clockwiseAngle;                     // Turn Right (negative)
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff)
    {
        double steer = error * PCoeff;
        if (steer < 0.0 && steer > -MIN_TURN_SPEED)
            steer = -MIN_TURN_SPEED;
        else if (steer > 0.0 && steer < MIN_TURN_SPEED)
            steer = MIN_TURN_SPEED;
        return Range.clip(error * PCoeff, -TURN_SPEED, TURN_SPEED);
    }


    public void encoderDrive(double inches, double timeoutS)
    {
        final double CLICKS_PER_REV = 383.6;    // Motor Encoder ticks for GoBilda 435RPM
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double CLICKS_PER_INCH = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        int newLeftTargetFront;
        int newRightTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;

        inches = -inches;


        hardware.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newLeftTargetFront = hardware.leftFront.getCurrentPosition() + (int) (inches * CLICKS_PER_INCH);
            newRightTargetFront = hardware.rightFront.getCurrentPosition() + (int) (inches * CLICKS_PER_INCH);
            newLeftTargetBack = hardware.leftBack.getCurrentPosition() + (int) (inches * CLICKS_PER_INCH);
            newRightTargetBack = hardware.rightBack.getCurrentPosition() + (int) (inches * CLICKS_PER_INCH);
            hardware.leftFront.setTargetPosition(newLeftTargetFront);
            hardware.rightFront.setTargetPosition(newRightTargetFront);
            hardware.leftBack.setTargetPosition(newLeftTargetBack);
            hardware.rightBack.setTargetPosition(newRightTargetBack);


            // Turn On RUN_TO_POSITION
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //rampUp(hardware.leftFront, hardware.rightFront, hardware.leftBack, hardware.rightBack);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (hardware.leftFront.isBusy() && hardware.rightFront.isBusy() && hardware.leftBack.isBusy() && hardware.rightBack.isBusy()))
            {

                if (newLeftTargetBack - hardware.leftBack.getCurrentPosition() <= 3 * CLICKS_PER_INCH
                        || newRightTargetBack - hardware.rightBack.getCurrentPosition() <= 3 * CLICKS_PER_INCH)
                {
                    //rampDown(hardware.leftFront, hardware.rightFront, hardware.leftBack, hardware.rightBack);
                }
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTargetFront, newRightTargetFront);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        hardware.leftFront.getCurrentPosition(),
                        hardware.rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            hardware.leftFront.setPower(0);
            hardware.rightFront.setPower(0);
            hardware.leftBack.setPower(0);
            hardware.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private double rampPower(double currentPower, double targetPower, double maxPower, double changeRate)
    {
        double power = targetPower;
        if (targetPower - currentPower > changeRate)
            power = currentPower + changeRate;
        if (currentPower - targetPower > changeRate)
            power = currentPower - changeRate;
        return Range.clip(power, -maxPower, maxPower);
    }


    private void driveEncoderPID(double DistanceIN)
    {

        final double CLICKS_PER_REV = 383.6;    // Motor Encoder ticks for GoBilda 435RPM
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double GearRatio = 2;
        final double CLICKS_PER_INCH = CLICKS_PER_REV * GearRatio / (Math.PI * WHEEL_DIAMETER_INCHES);
        final double fullSpeedDistance = 100 * CLICKS_PER_INCH;
        final double MOVE_ENCODER_TOLERANCE = 10;

        double leftFrontTargetEncoderValue = hardware.leftFront.getCurrentPosition() + DistanceIN * CLICKS_PER_INCH;
        double rightFrontTargetEncoderValue = hardware.rightFront.getCurrentPosition() + DistanceIN * CLICKS_PER_INCH;
        double leftBackTargetEncoderValue = hardware.leftBack.getCurrentPosition() + DistanceIN * CLICKS_PER_INCH;
        double rightBackTargetEncoderValue = hardware.rightBack.getCurrentPosition() + DistanceIN * CLICKS_PER_INCH;

        double leftBackError = leftBackTargetEncoderValue - hardware.leftBack.getCurrentPosition();
        double leftFrontError = leftFrontTargetEncoderValue - hardware.leftFront.getCurrentPosition();
        double rightBackError = rightBackTargetEncoderValue - hardware.rightBack.getCurrentPosition();
        double rightFrontError = rightFrontTargetEncoderValue - hardware.rightFront.getCurrentPosition();

        while (leftBackPID.checkTimeOut(leftBackError))
        {
            // Calculate error
            leftBackError = leftBackTargetEncoderValue - hardware.leftBack.getCurrentPosition();
            leftFrontError = leftFrontTargetEncoderValue - hardware.leftFront.getCurrentPosition();
            rightBackError = rightBackTargetEncoderValue - hardware.rightBack.getCurrentPosition();
            rightFrontError = rightFrontTargetEncoderValue - hardware.rightFront.getCurrentPosition();


            double driveSpeed = saturate(leftBackPID.getSpeed(leftBackError) / fullSpeedDistance);


            hardware.leftFront.setPower(driveSpeed);
            hardware.rightFront.setPower(driveSpeed);
            hardware.leftBack.setPower(driveSpeed);
            hardware.rightBack.setPower(driveSpeed);

            telemetry.addData(">", leftBackPID.printStuff(leftBackError));
            telemetry.update();

            sleep(50);
        }

        // Stop all motion;
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
    }

    private void gyroTurnPID(double angle)
    {
        double turnSpeed;
        double gyroError = angle - getCurrentAngle();
        double gyroTolerance = 0;
        double fullSpeedAngle = 480;
        double left = 0;
        double right = 0;

        if (angle < 0)
        {
            angle += 360;
        }

        while (gyroPID.checkTimeOutGyro(gyroError))
        {
            gyroError = Math.abs(getCurrentAngle() - angle);

            if (getCurrentAngle() < angle)
            {
                left = gyroError;
                right = 360 - gyroError;
            } else
            {
                left = 360 - gyroError;
                right = gyroError;
            }

            if (left < right)
            {
                turnSpeed = gyroPID.getSpeed(left) / fullSpeedAngle;

                turnSpeed = saturate(turnSpeed);

                hardware.leftFront.setPower(-turnSpeed);
                hardware.leftBack.setPower(-turnSpeed);

                hardware.rightFront.setPower(turnSpeed);
                hardware.rightBack.setPower(turnSpeed);
            } else
            {
                turnSpeed = gyroPID.getSpeed(right) / fullSpeedAngle;

                turnSpeed = saturate(turnSpeed);

                hardware.leftFront.setPower(turnSpeed);
                hardware.leftBack.setPower(turnSpeed);

                hardware.rightFront.setPower(-turnSpeed);
                hardware.rightBack.setPower(-turnSpeed);
            }


            //gyroError = Math.abs(angle - getCurrentAngle());

        }

        // Stop all motion;
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
    }

    private void strafeForTime(double speed, double sec, boolean left)
    {
        runtime.reset();
        if (left)
        {
            while (runtime.seconds() < sec)
            {
                strafe(speed);
            }
        }
        else
        {
            while (runtime.seconds() < sec)
            {
                strafe(-speed);
            }
        }
    }

    public void PIDMotorMove(double inches, DcMotor motor)
    {
        final double COUNTS_PER_MOTOR_REV = 1497.325;    // Motor Encoder for lift system
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 1.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);

        int newTarget;
        double speed;
        double error;


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newTarget = motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            motor.setTargetPosition(newTarget);


            error = newTarget - motor.getCurrentPosition();

            speed = saturate(liftPID.getSpeed(error));


            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (motor.isBusy()))
            {

                error = newTarget - motor.getCurrentPosition();

                speed = saturate(liftPID.getSpeed(error));
                motor.setPower(speed);
            }

            // Stop all motion;
            motor.setPower(0);

            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public void encoderMove(double speed, double inches, double timeoutS, DcMotor motor)
    {
        int newTarget;

        final double COUNTS_PER_MOTOR_REV = 1497.325;    // Motor Encoder for lift system
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 1.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newTarget = motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            motor.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor.isBusy()))
            {

            }

            // Stop all motion;
            motor.setPower(0);


            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    /* void printColorDistance() {
         // convert the RGB values to HSV values.
         // multiply by the SCALE_FACTOR.
         // then cast it back to int (SCALE_FACTOR is a double)
         Color.RGBToHSV((int) (hardware.sensorColor.red() * hardware.SCALE_FACTOR),
                 (int) (hardware.sensorColor.green() * hardware.SCALE_FACTOR),
                 (int) (hardware.sensorColor.blue() * hardware.SCALE_FACTOR),
                 hardware.hsvValues);

         // send the info back to driver station using telemetry function.
         telemetry.addData("Distance (cm)",
                 String.format(Locale.US, "%.02f", hardware.sensorDistance.getDistance(DistanceUnit.CM)));
         telemetry.addData("Alpha", hardware.sensorColor.alpha());
         telemetry.addData("Red  ", hardware.sensorColor.red());
         telemetry.addData("Green", hardware.sensorColor.green());
         telemetry.addData("Blue ", hardware.sensorColor.blue());
         telemetry.addData("Hue", hardware.hsvValues[0]);

         // change the background color to match the color detected by the RGB sensor.
         // pass a reference to the hue, saturation, and value array as an argument
         // to the HSVToColor method.
         hardware.relativeLayout.post(new Runnable() {
             public void run() {
                 hardware.relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hardware.values));
             }
         });

         // Set the panel back to the default color
         hardware.relativeLayout.post(new Runnable() {
             public void run() {
                 hardware.relativeLayout.setBackgroundColor(Color.WHITE);
             }
         });
     }
          */
    double getCurrentAngle()
    {

        currentAngle = hardware.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;

        if (currentAngle < 0)
        {
            return currentAngle + 360;
        } else
        {
            return currentAngle;
        }


    }

    double saturate(double speed)
    {
        if (speed > 1.0)
        {
            return 1.0;
        } else if (speed < -1.0)
        {
            return -1.0;
        } else
        {
            return speed;
        }
    }

    void motorRamp(DcMotor motor)
    {
        double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
        int CYCLE_MS = 50;     // period of each cycle
        double MAX_FWD = 0.5;     // Maximum FWD power applied to motoramp the motors, according to the rampUp variable.// Ramp the motors, according to the rampUp variable.
        double MAX_REV = -0.5;     // Maximum REV power applied to motorif (rampUp) {
        // Keep stepping up until we hit the max value.
        // Define class members    power += INCREMENT ;

        double power = 0;
        boolean rampUp = true;

        if (power >= MAX_FWD)
        {
            power = MAX_FWD;
            rampUp = !rampUp;   // Switch ramp direction
        } else
        {
            // Keep stepping down until we hit the min value.
            power -= INCREMENT;
            if (power <= MAX_REV)
            {
                power = MAX_REV;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }

        motor.setPower(power);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
