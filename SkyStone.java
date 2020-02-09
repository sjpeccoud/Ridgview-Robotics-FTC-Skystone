package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class SkyStone extends LinearOpMode {
    private static final double P_TURN_COEFF = 10.0;
    private static final double I_TURN_COEFF = 0.05;
    private static final double D_TURN_COEFF = 0.01;

    private static final double P_DRIVE_COEFF = 5.0;
    private static final double I_DRIVE_COEFF = 0.0;
    private static final double D_DRIVE_COEFF = 0.15;

    private static final double P_LIFT_COEFF = 0.07;
    private static final double I_LIFT_COEFF = 0.07;
    private static final double D_LIFT_COEFF = 0.07;

    private static final double HEADING_THRESHOLD = 0.5;
    private static final double MIN_TURN_SPEED = 0.3;
    private static final double TURN_SPEED = 0.5;

    private static final double P_TURN_COEFF2 = 0.06;

    double MOVE_ENCODER_TOLERANCE = 10;

    // Declare Timers
    public ElapsedTime runtime = new ElapsedTime();

    // Servo Positions
    final double ROTATE_END_MIDDLE = 0.57;
    final double ROTATE_END_BACK = 0.8;
    final double ROTATE_END_FORWARD = 0.1;

    final double ROTATE_RIGHT = 0.84;
    final double ROTATE_MIDDLE = 0.6;
    final double ROTATE_LEFT = 0.26;

    final double PINCH_DOWN = 1.0;
    final double PINCH_UP = 0.6;

    final double BLOCK_HEIGHT = 1.29;

    final double CLAMP_RELEASED = 0.95;
    final double CLAMP_ENGAGED = 0.67;

    final double hook1UpPos = 0.1;
    final double hook1DownPos = 0.6;

    final double hook2UpPos = 0.6;
    final double hook2DownPos = 0.1;

    final double capRotateHome = 0.95;
    final double capRotateDrop = 0.1;

    final double capClamp = 0.3;
    final double capRelease = 0.6;

    final double rotateArmHome = 0.3;
    final double rotateArmInit = 0.25;
    final double rotateArmALilOut = 0.4;
    final double rotateArmOut = 0.59;

    final double autoClampDown = 0.5;
    final double autoClampUp = 1;

    // Declare Variables
    int level = 1;
    boolean armPositionState;    //true == Home
    double currentAngle;
    boolean aValue = false;
    boolean bValue = false;
    boolean speed = false;
    boolean readyToRaiseAgain = true;
    double[] dataX = new double[20];
    double[] dataY = new double[20];
    boolean prevIntakeState = false;
    boolean is_engaged = false;
    String cubePos = "Middle";

    HardwareSkystone hardware = new HardwareSkystone();

    PID_Controller leftBackPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
    PID_Controller leftFrontPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
    PID_Controller rightBackPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);
    PID_Controller rightFrontPID = new PID_Controller(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);

    PID_Controller gyroPID = new PID_Controller(P_TURN_COEFF, I_TURN_COEFF, D_TURN_COEFF);

    PID_Controller liftPID = new PID_Controller(P_LIFT_COEFF, I_LIFT_COEFF, D_LIFT_COEFF);

    /*
     * VUFORIA
     *
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
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AXX6Dh7/////AAABmbgZiIyG1E9ThQ6TXfPkd8M6i6PCEXDZjkt2SpXyFdQmm+CuRQp" +
                    "6A9osXLHPk1eusykK9vUsTv4XVG+T9Ikgor/aU0YALMCsVj6t0igpi5T31wEak6" +
                    "wMpvaJTSS1uEJ99G99yLh6TsHIMHBRICuGERoSBfnRwowu2/FYIv8eFsN52R++XyF" +
                    "nC8DGk/XIZSeWb6v5adayCKLUaUSBXeuTAZLHj17j0TJojO3osifAfe2JmHwKkFYozZ" +
                    "kkzt+eRkImzOFPkcnE3D2otxVbbPNqUtns0lo0Dh7Ifd0p+M4MfWeWXHf/kN19Q7pIHqf3ie" +
                    "nh58D/8Y+zsoSHtoc/62SgM+K+oVhscYHu/DpIVAFo0IXA";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    private static final float mmPerInch = 25.4f;


    /////////// All Methods Below////////////


    //The Red Autonomous Starts
    public void redLineStartLeft()
    {
        driveEncoderPID(21);

        autoArmLower();

        strafeEncoder(0.4, -6, 1);

        autoArmIntakeClamp();

        strafeEncoder(0.4, 3, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-85);
        gyroTurn(0.5, 0, 250);



        strafeEncoder(0.4, -10, 1);
        autoArmIntakeDrop();
        strafeEncoder(0.4, 10, 1);


        gyroTurn(0.5, 0, 250);
        driveEncoderPID(73);
        gyroTurn(0.5, 0, 250);

        autoArmLower();
        strafeEncoder(0.4, -9, 1);
        autoArmIntakeClamp();
        strafeEncoder(0.4, 5, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-85);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, -12, 1);
        autoArmIntakeDrop();
    }

    public void redLineStartMiddle()
    {
        //was 15
        driveEncoderPID(13);

        autoArmLower();
        sleep(500);

        strafeEncoder(0.4, -5, 1);

        autoArmIntakeClamp();

        //was 5
        strafeEncoder(0.4, 4, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-80);
        gyroTurn(0.5, 0, 250);



        strafeEncoder(0.4, -10, 1);
        autoArmIntakeDrop();
        strafeEncoder(0.4, 10, 1);


        gyroTurn(0.5, 0, 250);
        driveEncoderPID(65);
        gyroTurn(0.5, 0, 250);

        autoArmLower();
        strafeEncoder(0.4, -7, 1);
        autoArmIntakeClamp();
        strafeEncoder(0.4, 7, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-80);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, -12, 1);
        autoArmIntakeDrop();
    }

    public void redLineStartRight()
     {
        driveEncoderPID(5);

        autoArmLower();

        strafeEncoder(0.4, -5, 1);

        autoArmIntakeClamp();

        strafeEncoder(0.4, 5, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-68);
        gyroTurn(0.5, 0, 250);



        strafeEncoder(0.4, -7, 1);
        autoArmIntakeDrop();
        strafeEncoder(0.4, 7, 1);


        gyroTurn(0.5, 0, 250);
        driveEncoderPID(90);
        gyroTurn(0.5, 0, 250);

        //strafeEncoder(0.4, 2, 1);
        autoArmLower();
        //sleep(500);
        strafeEncoder(0.4, -10, 1);
        autoArmIntakeClamp();
        strafeEncoder(0.4, 8, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-105);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, -12, 1);
        autoArmIntakeDrop();
    }



    //The Blue Autonomous Starts
    public void blueLineStartLeft()
    {
        driveEncoderPID(13);

        autoArmLower();

        strafeEncoder(0.4, -5, 1);

        autoArmIntakeClamp();

        strafeEncoder(0.4, 5, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(65);
        gyroTurn(0.5, 0, 250);



        strafeEncoder(0.4, -10, 1);
        autoArmIntakeDrop();
        strafeEncoder(0.4, 10, 1);


        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-94);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, 2, 1);
        autoArmLower();
        strafeEncoder(0.4, -4, 1);
        autoArmIntakeClamp();
        strafeEncoder(0.4, 5, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(95);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, -13, 1);
        autoArmIntakeDrop();
    }

    public void blueLineStartMiddle()
    {
        driveEncoderPID(6);

        autoArmLower();
        strafeEncoder(0.4, -5, 1);
        autoArmIntakeClamp();

        strafeEncoder(0.4, 5, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(73);
        gyroTurn(0.5, 0, 250);


        //strafeEncoder(0.4, 2, 1);
        strafeEncoder(0.4, -10, 1);
        autoArmIntakeDrop();
        strafeEncoder(0.4, 10, 1);


        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-101);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, 2, 1);
        autoArmLower();
        strafeEncoder(0.4, -8, 1);
        autoArmIntakeClamp();
        strafeEncoder(0.4, 8, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(107);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, -13, 1);
        autoArmIntakeDrop();
    }

    public void blueLineStartRight()
    {
        driveEncoderPID(-3);
        autoArmLower();
        strafeEncoder(0.4, -5, 1);

        autoArmIntakeClamp();

        strafeEncoder(0.4, 5, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(82);
        gyroTurn(0.5, 0, 250);



        strafeEncoder(0.4, -10, 1);
        autoArmIntakeDrop();
        strafeEncoder(0.4, 10, 1);


        gyroTurn(0.5, 0, 250);
        driveEncoderPID(-111);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, 3, 1);
        autoArmLower();
        strafeEncoder(0.4, -5, 1);
        autoArmIntakeClamp();
        strafeEncoder(0.4, 6, 1);

        gyroTurn(0.5, 0, 250);
        driveEncoderPID(115);
        gyroTurn(0.5, 0, 250);

        strafeEncoder(0.4, -14, 1);
        autoArmIntakeDrop();
    }




    public void resetGyro(boolean command)
    {
        if(command)
        {
            hardware.resetGyro();
        }
    }

    public void autoArmDropControl(boolean command)
    {
        /*
        if(command)
        {
            hardware.intakeServo.setPosition(autoClampUp);
            sleep(300);
            hardware.rotateIntakeServo.setPosition(rotateArmOut);
        }
         */

        if (command)
        {
            if (bValue)
            {
                hardware.intakeServo.setPosition(autoClampUp);
                sleep(300);
                hardware.rotateIntakeServo.setPosition(rotateArmOut);
                bValue = false;
                sleep(100);

            } else
            {
                hardware.intakeServo.setPosition(autoClampDown);
                sleep(300);
                hardware.rotateIntakeServo.setPosition(rotateArmHome);
                sleep(100);
                bValue = true;
            }
        }

    }

    public void autoArmIntakeHome()
    {
        hardware.rotateIntakeServo.setPosition(rotateArmHome);
        hardware.intakeServo.setPosition(autoClampDown);
    }

    public void autoArmIntakeInit()
    {
        hardware.rotateIntakeServo.setPosition(rotateArmInit);
        hardware.intakeServo.setPosition(autoClampDown);
    }

    public void autoArmIntakeClamp()
    {
            hardware.intakeServo.setPosition(autoClampDown);
            sleep(500);
            autoArmIntakeHome();


    }

    public void autoArmIntakeDrop()
    {
        hardware.rotateIntakeServo.setPosition(rotateArmOut);
        sleep(300);
        hardware.intakeServo.setPosition(autoClampUp);
        sleep(300);
        hardware.rotateIntakeServo.setPosition(rotateArmHome);
        sleep(100);
        hardware.intakeServo.setPosition(autoClampDown);

    }

    public void autoArmLower()
    {
        hardware.intakeServo.setPosition(autoClampUp);
        sleep(200);
        hardware.rotateIntakeServo.setPosition(rotateArmOut);
        //sleep(200);
    }

    public void mecanumControl(float command)
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



        if(command > 0)
        {
            hardware.leftFront.setPower(frontLeftPower * 0.4);
            hardware.rightFront.setPower(frontRightPower * 0.4);
            hardware.leftBack.setPower(backLeftPower * 0.4);
            hardware.rightBack.setPower(backRightPower * 0.4);
        }
        else
        {
            // was 0.75
            hardware.leftFront.setPower(frontLeftPower * 0.80);
            hardware.rightFront.setPower(frontRightPower * 0.80);
            hardware.leftBack.setPower(backLeftPower * 0.80);
            hardware.rightBack.setPower(backRightPower * 0.80);
        }

    }

    public void mecanumFineControl()
    {
        double drive;   // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotate;  // Power for rotating the robot


        drive = -gamepad2.left_stick_y;  // Negative because the gamepad is weird
        strafe = -gamepad2.left_stick_x;
        rotate = gamepad2.right_stick_x;

        // You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower = drive - strafe + rotate;
        double backLeftPower = drive + strafe + rotate;
        double frontRightPower = drive + strafe - rotate;
        double backRightPower = drive - strafe - rotate;

            hardware.leftFront.setPower(frontLeftPower * 0.2);
            hardware.rightFront.setPower(frontRightPower * 0.2);
            hardware.leftBack.setPower(backLeftPower * 0.2);
            hardware.rightBack.setPower(backRightPower * 0.2);

    }

    public void capstoneHome()
    {
        hardware.rotateCapstoneServo.setPosition(capRotateHome);
        hardware.clampCapstoneServo.setPosition(capClamp);
    }

    public void capstoneInit()
    {
        hardware.rotateCapstoneServo.setPosition(capRotateDrop);
        hardware.clampCapstoneServo.setPosition(capClamp);
    }

    private void capstoneDrop()
    {
        hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
        hardware.rotateCapstoneServo.setPosition(capRotateDrop);
        clampEngage();
        hardware.rotateServo.setPosition(ROTATE_MIDDLE);
        sleep(1000);
        hardware.clampCapstoneServo.setPosition(capRelease);
        hardware.rotateIntakeServo.setPosition(rotateArmHome);

    }

    public void capstoneControl(boolean command)
    {
        if (command)
        {
            capstoneDrop();
            sleep(500);
            capstoneHome();
            armHome();
        }
    }

    public void intakeControl()
    {

        if (gamepad1.left_trigger <= gamepad1.right_trigger)
        {
            hardware.rotateServo.setPosition(ROTATE_RIGHT);
            hardware.rightIntake.setPower(-gamepad1.right_trigger);
            hardware.leftIntake.setPower(-gamepad1.right_trigger);
        } else
        {
            clampRelease();
            hardware.rotateServo.setPosition(ROTATE_RIGHT + 0.2);
            hardware.rightIntake.setPower(gamepad1.left_trigger * 0.75);
            hardware.leftIntake.setPower(gamepad1.left_trigger * 0.75);
        }

    }

    public void intakeIn(double speed)
    {
        hardware.rightIntake.setPower(-speed);
        hardware.leftIntake.setPower(-speed);
    }

    public void intakeOut(double speed)
    {
        hardware.rightIntake.setPower(speed);
        hardware.leftIntake.setPower(speed);
    }

    public void hooksDown()
    {
        hardware.hook1.setPosition(hook1DownPos);
        hardware.hook2.setPosition(hook2DownPos);
    }

    public void hooksUp()
    {
        hardware.hook1.setPosition(hook1UpPos);
        hardware.hook2.setPosition(hook2UpPos);
    }

    public void hooksControl (boolean command)
    {
        if (command)
        {
            if (aValue)
            {
                hooksUp();
                aValue = false;
                sleep(500);

            } else
            {
                hooksDown();
                sleep(500);
                aValue=true;
            }
        }
    }

    public void levelControl()
    {
        if(gamepad2.dpad_down)
        {
            level = 1;
        }
        else if(gamepad2.dpad_left)
        {
            level = 2;
        }
        else if(gamepad2.dpad_up)
        {
            level = 3;
        }
        else if(gamepad2.dpad_right)
        {
            level = 4;
        }
        else if(gamepad2.left_bumper)
        {
            level = 5;
        }
        else if(gamepad2.right_bumper)
        {
            level = 6;
        }

        telemetry.addData("Level: ", level);
        telemetry.update();
    }

    // Slows a Servo's speed down to a slower constant speed
    private void servoSlow(Servo servo, double pos)
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



    void autoStackBack()
    {
        boolean exit = false;

        if (level <= 8 && level >= 5)
        {
            //Strafe until aligned
            runtime.reset();
            telemetry.addData("Dist Left: ", hardware.distanceSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Dist Right: ", hardware.distanceSensorRight.getDistance(DistanceUnit.CM));
            telemetry.update();
            //sleep(1000);
            straightForTime(-0.3, 0.25);
            while(hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > 6 || hardware.distanceSensorRight.getDistance(DistanceUnit.CM) > 6)
            {
                if (hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > hardware.distanceSensorRight.getDistance(DistanceUnit.CM))
                {
                    strafe(-0.17);
                }
                else
                {
                    strafe(0.17);
                }

                if(gamepad2.a)
                {
                    exit = true;
                    break;
                }
            }

            strafe(0);

            if (!exit)
            {
                hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
                //Pinch Block
                pinchDown();
                sleep(200);
                clampRelease();
                sleep(400);


                //Gets Ready to Drop Block
                encoderMove(0.7, ((BLOCK_HEIGHT * (level - 1)) + 1), 15, hardware.liftMotor);
                armOutBackFast();
                sleep(500);

                //Sets the Block Down a Little
                encoderMove(0.1, -1.5, 15, hardware.liftMotor);
                sleep(250);

                //Drops Block
                pinchUp();
                sleep(100);

                //Resets Lift System
                encoderMove(0.3, 1.5, 15, hardware.liftMotor);
                sleep(100);
                armHome();
                sleep(500);
                encoderMove(0.7, -((BLOCK_HEIGHT * (level - 1)) + 0.97), 15, hardware.liftMotor);
                level++;
            }
        }

        if (level <= 4 && level > 1)
        {
            //Strafe until aligned
            runtime.reset();
            telemetry.addData("Dist Left: ", hardware.distanceSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Dist Right: ", hardware.distanceSensorRight.getDistance(DistanceUnit.CM));
            telemetry.update();
            //sleep(1000);
            straightForTime(0.3, 0.25);
            while(hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > 6 || hardware.distanceSensorRight.getDistance(DistanceUnit.CM) > 6)
            {
                if (hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > hardware.distanceSensorRight.getDistance(DistanceUnit.CM))
                {
                    strafe(-0.17);
                }
                else
                {
                    strafe(0.17);
                }

                if(gamepad2.a)
                {
                    exit = true;
                    break;
                }
            }

            strafe(0);

            if (!exit)
            {
                hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
                //Pinch Block
                pinchDown();
                sleep(200);
                clampRelease();
                sleep(400);


                //Gets Ready to Drop Block
                encoderMove(0.7, ((BLOCK_HEIGHT * (level - 1)) + 1), 15, hardware.liftMotor);
                armOutBackFast();
                sleep(500);

                //Sets the Block Down a Little
                encoderMove(0.15, -1.5, 15, hardware.liftMotor);
                sleep(250);

                //Drops Block
                pinchUp();
                sleep(100);

                //Resets Lift System
                encoderMove(0.3, 1.5, 15, hardware.liftMotor);
                sleep(100);
                armHome();
                sleep(500);
                encoderMove(0.7, -((BLOCK_HEIGHT * (level - 1)) + 0.97), 15, hardware.liftMotor);
                level++;
            }
        }

        if (level == 1)
        {
            hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
            //Pinch Block
            pinchDown();
            sleep(200);
            clampRelease();
            sleep(400);


            //Gets Ready to Drop Block
            encoderMove(0.7, 1.7, 15, hardware.liftMotor);
            armOutBackSlow();
            //sleep(500);

            //Sets the Block Down a Little
            encoderMove(0.3, -1.7, 15, hardware.liftMotor);
            //sleep(500);

            //Drops Block
            pinchUp();
            sleep(100);

            //Resets Lift System
            encoderMove(0.3, 1.7, 15, hardware.liftMotor);
            sleep(100);
            armHome();
            sleep(100);
            encoderMove(0.7, -1.67, 15, hardware.liftMotor);
            level++;
        }
        hardware.rotateIntakeServo.setPosition(rotateArmHome);
    }

    void raiseToStack(boolean command)
    {
        if(command && readyToRaiseAgain)
        {
            hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
            if(level == 1)
            {
                //Pinch Block
                pinchDown();
                runtime.reset();
                while(runtime.seconds() < 0.3)
                {
                    mecanumControl(gamepad2.right_trigger);
                }
                clampRelease();
                //sleep(400);


                //Gets Ready to Drop Block
                encoderMove(0.7, 1.7, 15, hardware.liftMotor);
            }
            if(level >= 2)
            {
                //Pinch Block
                pinchDown();
                runtime.reset();
                while(runtime.seconds() < 0.3)
                {
                    mecanumControl(gamepad2.right_trigger);
                }
                clampRelease();
                //sleep(400);


                //Gets Ready to Drop Block
                encoderMove(0.7, ((BLOCK_HEIGHT * (level - 1)) + 1), 5, hardware.liftMotor);
            }

            readyToRaiseAgain = false;
        }

        hardware.rotateIntakeServo.setPosition(rotateArmHome);
    }

    void placeAndReturn(boolean command)
    {
        if(command && !readyToRaiseAgain)
        {
            boolean exit = false;
            hardware.rotateIntakeServo.setPosition(rotateArmALilOut);

            if(level >= 5)
            {
                straightForTime(0.3, 0.25);
                while(hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > 6 || hardware.distanceSensorRight.getDistance(DistanceUnit.CM) > 6)
                {
                    if (hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > hardware.distanceSensorRight.getDistance(DistanceUnit.CM))
                    {
                        strafe(-0.17);
                    }
                    else
                    {
                        strafe(0.17);
                    }

                    if(gamepad2.a)
                    {
                        exit = true;
                        break;
                    }
                }

                strafe(0);

                if (!exit)
                {

                    armOutBackFast();
                    sleep(500);

                    //Sets the Block Down a Little
                    //was speed 0.06
                    encoderMove(0.07, -1.5, 15, hardware.liftMotor);
                    sleep(250);

                    while(!gamepad2.a)
                    {
                        mecanumFineControl();
                    }
                    //Drops Block
                    pinchUp();
                    sleep(100);

                    //Resets Lift System
                    encoderMove(0.3, 1.5, 15, hardware.liftMotor);
                    sleep(100);
                    armHome();
                    sleep(500);
                    encoderMove(0.3, -((BLOCK_HEIGHT * (level - 1)) + 1), 15, hardware.liftMotor);
                    level++;
                    readyToRaiseAgain = true;
                }
            }

            if(level >= 2 && level < 5)
            {
                straightForTime(0.3, 0.25);
                while(hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > 6 || hardware.distanceSensorRight.getDistance(DistanceUnit.CM) > 6)
                {
                    if (hardware.distanceSensorLeft.getDistance(DistanceUnit.CM) > hardware.distanceSensorRight.getDistance(DistanceUnit.CM))
                    {
                        strafe(-0.17);
                    }
                    else
                    {
                        strafe(0.17);
                    }

                    if(gamepad2.a)
                    {
                        exit = true;
                        break;
                    }
                }

                strafe(0);

                driveEncoderPID(-2);

                if (!exit)
                {
                    hardware.rotateIntakeServo.setPosition(rotateArmALilOut);

                    armOutBackFast();
                    sleep(500);

                    //Sets the Block Down a Little
                    //was speed 0.06
                    encoderMove(0.07, -1.25, 15, hardware.liftMotor);
                    //sleep(250);

                    //Drops Block
                    pinchUp();
                    sleep(100);

                    //Resets Lift System
                    encoderMove(0.3, 1.25, 15, hardware.liftMotor);
                    sleep(100);
                    armHome();
                    sleep(500);
                    encoderMove(0.3, -((BLOCK_HEIGHT * (level - 1)) + 1), 15, hardware.liftMotor);
                    level++;
                    readyToRaiseAgain = true;
                }
            }
            if (level == 1)
            {
                armOutBackSlow();
                //sleep(500);

                //Sets the Block Down a Little
                encoderMove(0.3, -1.7, 15, hardware.liftMotor);
                //sleep(500);

                //Drops Block
                //strafeEncoder(0.1, 1, 1);
                while(!gamepad2.a)
                {
                    mecanumFineControl();
                }
                pinchUp();
                sleep(100);

                //Resets Lift System
                encoderMove(0.3, 1.7, 15, hardware.liftMotor);
                sleep(100);
                armHome();
                sleep(100);
                encoderMove(0.3, -1.67, 15, hardware.liftMotor);
                level++;
                readyToRaiseAgain = true;
            }

            hardware.rotateIntakeServo.setPosition(rotateArmHome);

        }
    }



    public void autoStackSide()
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
            armOutSide();
            sleep(1500);

            //Sets the Block Down a Little
            encoderMove(0.3, -1, 15, hardware.liftMotor);
            sleep(100);

            //Drops Block
            pinchUp();
            sleep(250);

            //Resets Lift System
            encoderMove(0.3, 1, 15, hardware.liftMotor);
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
            encoderMove(0.7, 1, 15, hardware.liftMotor);
            armOutSide();
            sleep(1500);

            //Sets the Block Down a Little
            encoderMove(0.3, -1.3, 15, hardware.liftMotor);
            sleep(100);

            //Drops Block
            pinchUp();
            sleep(250);

            //Resets Lift System
            encoderMove(0.3, 1.3, 15, hardware.liftMotor);
            sleep(100);
            armHome();
            sleep(100);
            encoderMove(0.7, -1, 15, hardware.liftMotor);
            level++;
        }
    }

    public void autoStackBack(boolean command)
    {
        if (command)
        {
            autoStackBack();
        }
    }

    public void autoStackSide(boolean command)
    {
        if (command)
        {
            autoStackSide();
        }
    }

    public void pinchServoControl(boolean command)
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

    public void rotateEndServoControl(boolean command)
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

    public void liftSystemManualControl(boolean upCommand, boolean downCommand)
    {
        if (upCommand)
        {
            hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
            hardware.liftMotor.setPower(0.6);
            sleep(100);
        } else
        {
            hardware.rotateIntakeServo.setPosition(rotateArmHome);
            hardware.liftMotor.setPower(0);
        }
        if (downCommand)
        {
            hardware.rotateIntakeServo.setPosition(rotateArmALilOut);
            hardware.liftMotor.setPower(-0.6);
            sleep(100);
        } else
        {
            hardware.rotateIntakeServo.setPosition(rotateArmHome);
            hardware.liftMotor.setPower(0);
        }
    }

    public void armControl(boolean command)
    {

        if (command)
        {
            if (armPositionState)
            {
                armOutSide();
            } else
            {
                armHome();
            }
        }
    }

    public void armHome()
    {
        //get ready to collect another cube
        pinchUp();
        //rotate left
        hardware.rotateServo.setPosition(ROTATE_RIGHT);
        //towards back
        hardware.rotateEndServo.setPosition(ROTATE_END_FORWARD);
    }

    public void armInit()
    {
        //get ready to collect another cube
        pinchUp();
        //rotate left
        hardware.rotateServo.setPosition(ROTATE_RIGHT + 0.2);
        //towards back
        hardware.rotateEndServo.setPosition(ROTATE_END_FORWARD);
    }

    public void autoClamp()
    {
        if (hardware.sensorRange.getDistance(DistanceUnit.CM) < 5)
        {
            //sleep(200);
            clampEngage();
        }
        else
        {
            clampRelease();
        }
    }

    public void clampRelease()
    {
        hardware.clampServo.setPosition(CLAMP_RELEASED);
    }

    public void clampEngage()
    {
        hardware.clampServo.setPosition(CLAMP_ENGAGED);
    }

    public void clampControl(boolean command)
    {

        if (command)
        {
            clampEngage();
        } else if (gamepad1.x)
        {
            clampRelease();
        }
    }

    private void armOutBackFast()
    {
        pinchDown();
        hardware.rotateServo.setPosition(ROTATE_MIDDLE);
        sleep(400);
        hardware.rotateEndServo.setPosition(ROTATE_END_BACK);
        hardware.rotateServo.setPosition(ROTATE_RIGHT);

    }

    private void armOutBackSlow()
    {
        pinchDown();
        servoSlow(hardware.rotateServo, ROTATE_MIDDLE);
        servoSlow(hardware.rotateEndServo, ROTATE_END_BACK);
        servoSlow(hardware.rotateServo, ROTATE_RIGHT);
        //hardware.rotateServo.setPosition(ROTATE_MIDDLE);
        //sleep(400);
        //hardware.rotateEndServo.setPosition(ROTATE_END_BACK);
        //hardware.rotateServo.setPosition(ROTATE_RIGHT);

    }

    private void armOutSide()
    {
        pinchDown();
        servoSlow(hardware.rotateServo, ROTATE_LEFT);
        servoSlow(hardware.rotateEndServo, ROTATE_END_MIDDLE);
    }

    private void pinchUp()
    {
        //Drops Block
        hardware.pinchServo.setPosition(PINCH_UP);
    }

    private void pinchDown()
    {
        //Pinch Block
        hardware.pinchServo.setPosition(PINCH_DOWN);
    }

    // neg speed goes back
    private void straight(double speed)
    {
        speed = -speed;
        hardware.leftFront.setPower(speed);
        hardware.rightFront.setPower(speed);
        hardware.leftBack.setPower(speed);
        hardware.rightBack.setPower(speed);
    }

    // neg speed goes right
    private void strafe(double speed)
    {
        hardware.leftFront.setPower(-speed);
        hardware.rightFront.setPower(speed);
        hardware.leftBack.setPower(speed);
        hardware.rightBack.setPower(-speed);
    }

    // neg speed turns right
    private void turn(double speed)
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

    public void gyroTurn(double speed, double angle, double maxTimeMS)
    {

        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        if (angle < 0)
            angle += 360;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF2) && (turnTimer.milliseconds() < maxTimeMS))
        {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.addData(">:", "Gyro Turn is Looping");
            //telemetry.update();
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
            onHeading(0.5, angle, P_TURN_COEFF2);
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
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        hardware.leftFront.setPower(leftSpeed);
        hardware.leftBack.setPower(leftSpeed);
        hardware.rightFront.setPower(rightSpeed);
        hardware.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getCurrentAnglePos();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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


    public void driveEncoderPID(double DistanceIN)
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

            if(!opModeIsActive())
                break;
            //sleep(50);
        }

        // Stop all motion;
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
    }

    public void gyroTurnPID(double angle)
    {
        double turnSpeed;
        double gyroError = Math.abs(getCurrentAnglePos() - angle);
        double fullSpeedAngle = 480;
        double left = 0;
        double right = 0;
        double timeout = 1.0;
        double tolerance = 3;

        if (angle < 0)
        {
            angle += 360;
        }

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeout)
        {

            telemetry.addData("Heading: ", currentAngle);
            telemetry.update();

            gyroError = Math.abs(getCurrentAnglePos() - angle);

            if (getCurrentAnglePos() < angle)
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
                //turnSpeed = gyroPID.getSpeed(left);

                turnSpeed = saturate(turnSpeed);

                hardware.leftFront.setPower(-turnSpeed);
                hardware.leftBack.setPower(-turnSpeed);

                hardware.rightFront.setPower(turnSpeed);
                hardware.rightBack.setPower(turnSpeed);
            } else
            {
                turnSpeed = gyroPID.getSpeed(right) / fullSpeedAngle;
                //turnSpeed = gyroPID.getSpeed(right);

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

    public void cardinalDirectionTurn(boolean command)
    {
        if(command)
        {
            if(getCurrentAnglePos() >= 315 || getCurrentAnglePos() <= 45)
            {
                gyroTurnPID(0);
            }
            else if (getCurrentAnglePos() > 45 && getCurrentAnglePos() <= 135)
            {
                gyroTurnPID(90);
            }
            else if (getCurrentAnglePos() > 135 && getCurrentAnglePos() <= 225)
            {
                gyroTurnPID(180);
            }
            else
            {
                gyroTurnPID(270);
            }
        }
    }

    public void strafeForTime(double speed, double sec, boolean left)
    {
        runtime.reset();
        if (left)
        {
            while (runtime.seconds() < sec)
            {
                strafe(speed);
            }
        } else
        {
            while (runtime.seconds() < sec)
            {
                strafe(-speed);
            }
        }

        // Stop all motion;
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
    }

    //left is positive
    public void strafeEncoder(double speed, double inches, double timeoutS)
    {
        inches *= 2;
        int newLeftTargetFront;
        int newRightTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;

        final double CLICKS_PER_REV = 383.6;    // Motor Encoder ticks for GoBilda 435RPM
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double COUNTS_PER_INCH = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        //rightInches = -rightInches;

        hardware.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetFront = hardware.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTargetFront = hardware.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftTargetBack = hardware.leftBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTargetBack = hardware.rightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            hardware.leftFront.setTargetPosition(-newLeftTargetFront);
            hardware.rightFront.setTargetPosition(newRightTargetFront);
            hardware.leftBack.setTargetPosition(newLeftTargetBack);
            hardware.rightBack.setTargetPosition(-newRightTargetBack);


            // Turn On RUN_TO_POSITION
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            hardware.leftFront.setPower(speed);
            hardware.rightFront.setPower(speed);
            hardware.leftBack.setPower(speed);
            hardware.rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (hardware.leftFront.isBusy() && hardware.rightFront.isBusy() && hardware.leftBack.isBusy() && hardware.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTargetFront,  newRightTargetFront);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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

    public void straightForTime(double speed, double sec)
    {
        runtime.reset();
        while(runtime.seconds() < sec && opModeIsActive())
        {
            straight(speed);
        }

        // Stop all motion;
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftBack.setPower(0);
        hardware.rightBack.setPower(0);
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

        runtime.reset();
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

                mecanumControl(gamepad2.right_trigger);
            }

            // Stop all motion;
            motor.setPower(0);


            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    /*
    void printColorDistance() {
         // convert the RGB values to HSV values.
         // multiply by the SCALE_FACTOR.
         // then cast it back to int (SCALE_FACTOR is a double)
         Color.RGBToHSV((int) (hardware.sensorColor.red() * hardware.SCALE_FACTOR),
                 (int) (hardware.sensorColor.green() * hardware.SCALE_FACTOR),
                 (int) (hardware.sensorColor.blue() * hardware.SCALE_FACTOR),
                 hardware.hsvValues);

         // send the info back to driver station using telemetry function.
         telemetry.addData("Distance (cm)",
                 String.format(Locale.US, "%.02f", hardware.distanceSensorLeft.getDistance(DistanceUnit.CM)));
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


    public double getCurrentAnglePos()
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

    public double getCurrentAngleNeg()
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
    public void initVuforia() {
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
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
