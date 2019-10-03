package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;


@TeleOp(name="Skystone TeleOp", group="Linear Opmode")

public class SkystoneTeleOp extends LinearOpMode {


    private static final double CLICKS_PER_REV = 735;    // 723.24 larger is further
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_INCHES * 2.54;
    private static final double CLICKS_PER_INCH = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double CLICKS_PER_CM = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_CM);

    private static final double ENCODER_DRIVE_SPEED     = .2;
    private static final double ENCODER_ROTATE_SPEED    = .05;
    private static final double DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    private static final double TURN_SPEED              = 0.8;     // Nominal half speed for better accuracy.

    private static final double HEADING_THRESHOLD       = 0.5;      // As tight as we can make it with without gyrating back and forth
    private static final double P_TURN_COEFF            = 0.07;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF           = 0.03;     // Larger is more responsive, but also less stable
    private static final double MIN_TURN_SPEED          = .3;      // Minimum motor speed for active turn



    // Declare Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime optime = new ElapsedTime();


    // Declare All Sensor Variables
    BNO055IMU imu;  // IMU sensor included in Rev hub
    Orientation angles;
    Acceleration gravity;


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
                    "ARSCJJ7/////AAAAGfa2YWhnEk+RiQFFs0R0v70p2sftSWshcnJm+b" +
                    "g5TII63alEaaI/moTANc4ae8Pqj91QHeCch/Yo/Rgc0GPtqOwWrFTzIeE6DfG" +
                    "hL66igU5tx6jc6/TFg1NygSOnZCmJ0DUZvQFF7YYR0xcvWG33Z4tZmlX+Tlk2f+pDhfe308HlEqsnispfNrwtCd" +
                    "PWZ6oEJDK4TZMAp6YQ4bSAQn4XPUslEAVGclMxSBHCiEpcl6Ts1407nLBgIyL9acQlUKDoIVUGxooyqdaIM5OlwxRLNd" +
                    "MqkLNGmk95IkI2xFT19Td9t52evSC8O2bdNrSJnnX/w+ppZkIppwSktU8usnpz9NEkfNDrpgPYG/pE9xq0tAyZ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    // Declare Drive Motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    //private DcMotor leftIntake = null;
    //private DcMotor rightIntake = null;

    // Declare Servos
    private Servo hook1 = null;
    private Servo hook2 = null;

    // Declare Variables
    boolean aValue = false;
    boolean xBool = false;
    double[] dataX = new double[20];
    double[] dataY = new double[20];
    double[] dataHeading = new double[20];
    double totalError = 0;


    double hook1UpPos = 0.9;
    double hook1DownPos = 0.1;

    double hook2UpPos = 0.1;
    double hook2DownPos = 0.8;



    @Override
    public void runOpMode() {

        ////////////GAME INITIALIZE////////////////////////////////////////////////

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //leftIntake  = hardwareMap.get(DcMotor.class, "leftIntake");
        //rightIntake  = hardwareMap.get(DcMotor.class, "rightIntake");

        // Initialize Servos
        hook1 = hardwareMap.get(Servo.class, "hook1");
        hook2 = hardwareMap.get(Servo.class, "hook2");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Moves Servos to starting position
        hook1.setPosition(hook1UpPos);
        hook2.setPosition(hook2UpPos);

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize Vuforia START////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////

            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
             * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection   = CAMERA_CHOICE;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Load the data sets for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
            VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
            blueRearBridge.setName("Blue Rear Bridge");
            VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
            redRearBridge.setName("Red Rear Bridge");
            VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
            redFrontBridge.setName("Red Front Bridge");
            VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
            blueFrontBridge.setName("Blue Front Bridge");
            VuforiaTrackable red1 = targetsSkyStone.get(5);
            red1.setName("Red Perimeter 1");
            VuforiaTrackable red2 = targetsSkyStone.get(6);
            red2.setName("Red Perimeter 2");
            VuforiaTrackable front1 = targetsSkyStone.get(7);
            front1.setName("Front Perimeter 1");
            VuforiaTrackable front2 = targetsSkyStone.get(8);
            front2.setName("Front Perimeter 2");
            VuforiaTrackable blue1 = targetsSkyStone.get(9);
            blue1.setName("Blue Perimeter 1");
            VuforiaTrackable blue2 = targetsSkyStone.get(10);
            blue2.setName("Blue Perimeter 2");
            VuforiaTrackable rear1 = targetsSkyStone.get(11);
            rear1.setName("Rear Perimeter 1");
            VuforiaTrackable rear2 = targetsSkyStone.get(12);
            rear2.setName("Rear Perimeter 2");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsSkyStone);

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

            // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
            // Rotated it to to face forward, and raised it to sit on the ground correctly.
            // This can be used for generic target-centric approach algorithms
            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            //Set the position of the bridge support targets with relation to origin (center of field)
            blueFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

            blueRearBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

            redFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, -bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

            redRearBridge.setLocation(OpenGLMatrix
                    .translation(bridgeX, -bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

            //Set the position of the perimeter targets with relation to origin (center of field)
            red1.setLocation(OpenGLMatrix
                    .translation(quadField, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            red2.setLocation(OpenGLMatrix
                    .translation(-quadField, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            front1.setLocation(OpenGLMatrix
                    .translation(-halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

            front2.setLocation(OpenGLMatrix
                    .translation(-halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

            blue1.setLocation(OpenGLMatrix
                    .translation(-quadField, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            blue2.setLocation(OpenGLMatrix
                    .translation(quadField, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            rear1.setLocation(OpenGLMatrix
                    .translation(halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

            rear2.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            //
            // Create a transformation matrix describing where the phone is on the robot.
            //
            // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
            // Lock it into Portrait for these numbers to work.
            //
            // Info:  The coordinate frame for the robot looks the same as the field.
            // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
            // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
            //
            // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
            // pointing to the LEFT side of the Robot.
            // The two examples below assume that the camera is facing forward out the front of the robot.

            // We need to rotate the camera around it's long axis to bring the correct camera forward.
            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            // Rotate the phone vertical about the X axis if it's in portrait mode
            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90 ;
            }

            // Next, translate the camera lens to where it is on the robot.
            // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
            final float CAMERA_FORWARD_DISPLACEMENT  = 9.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
            final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
            parametersG.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parametersG.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parametersG.loggingEnabled      = true;
            parametersG.loggingTag          = "IMU";
            parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            boolean initialized = imu.initialize(parametersG);
            sleep(2000);

            // Set up our telemetry dashboard
            telemetry.update();

            telemetry.addData("After composeTelemetry", "IMU initialized: %b", false);
            telemetry.update();
            sleep(2000);


            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize Vuforia END//////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////



        telemetry.addData(">", "Servos Initialized");
        telemetry.update();


        telemetry.addData(">", "Robot Ready. Waiting for Start.");
        telemetry.update();


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        sleep(2000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        ///////////////GAME START////////////////////////////////


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // Get Current Heading
            //angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

            telemetry.addData("current Angle: ", imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
            telemetry.update();

            // Method that controls the mecanum wheels
            mecanumControl();


            // Press A on Gamepad 1 Lowers Hooks
            hookControl();

       /* if (gamepad1.x) {
            leftIntake.setPower(1.0);
            rightIntake.setPower(-1.0);
        }*/

            if (gamepad1.x)
            {
                encoderDrive(0.3, 10, 10, 10);
            }


            ///////////////////////////////////////////////////////////////////////////////////////////////
            ////////// Vuforia SEARCH START ///////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////

            targetsSkyStone.activate();

                // Method that controls the mecanum wheels
                mecanumControl();


                // Press A on Gamepad 1 Lowers Hooks
                hookControl();



                    // check all the trackable targets to see which one (if any) is visible.
                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables)
                    {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                        {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible)
                    {

                        telemetry.addData("current Angle: ", imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
                        telemetry.update();

                        mecanumControl();
                        hookControl();
                        xAverage();
                        yAverage();

                        // Test all new code under Gamepad 1 Left Bumper
                        if (gamepad1.left_bumper)
                        {
                            gyroTurn(1, findAngle(0, 0), 10000);

                            telemetry.addData("Angle -x and -y: ", findAngle(0, 0));
                            telemetry.update();

                            sleep(500);

                            encoderDrive(0.3, findDistance(0, 0), findDistance(0, 0), 5);
                            telemetry.addData("Distance: ", findDistance(0, 0));
                            telemetry.update();
                            sleep(2000);
                        }


                        if (gamepad1.right_bumper)
                        {

                            // gyroTurn(1, findAngle(100, 100), 3000);
                            //telemetry.addData("Angle -y : ", findAngle(0, 0));
                            //telemetry.addData("Angle Megean: ", findAngleMegan(10, 10));
                            // telemetry.addData("Distance to point: ", findDistance(10, 10));

                            driveToPoint(30, 0, true);
                            telemetry.update();
                            sleep(500);
                        }

                        // express position (translation) of robot in inches.
                        //VectorF translation = lastLocation.getTranslation();
                        VectorF translation = lastLocation.getTranslation();


                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        //Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    }
                    else
                    {
                        telemetry.addData("Visible Target", "none");
                    }

                    sleep(50);
                }

            }


            ///////////////////////////////////////////////////////////////////////////////////////////////
            ////////// Vuforia SEARCH END /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////

            //sleep(50);




    /////////// All Methods Below////////////

    void mecanumControl ()
    {
        double drive;   // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotate;  // Power for rotating the robot


        drive = gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

// You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }

    void hookControl ()
    {
        if (gamepad1.a)
        {
            if (!aValue)
            {
                hook1.setPosition(hook1DownPos);
                hook2.setPosition(hook2DownPos);
                xBool = true;
                telemetry.addData(">", "Hook Down");
            }
            if (aValue)
            {
                hook1.setPosition(hook1UpPos);
                hook2.setPosition(hook2UpPos);
                xBool = false;
                telemetry.addData(">", "Hook Up");
            }
            aValue = xBool;
            telemetry.update();

        }
    }

    double xAverage ()
    {
        VectorF translation = lastLocation.getTranslation();

        double sumX = 0;
        double averageX;

        for (int i = 0; i < dataX.length-1; i++)
        {
            dataX[i] = dataX[i+1];
        }

        // At dataX slot 19 input the new data point
        dataX[dataX.length - 1] =  translation.get(0) / mmPerInch;

        String s = "";

            for (int i = 0; i < dataX.length; i++)
            {
                sumX += dataX[i];
                s += "," + dataX[i];
            }


            averageX = sumX / dataX.length;
            telemetry.addData("X Value Average: ", averageX);
            telemetry.update();
            return averageX;

    }


    double yAverage ()
    {
        VectorF translation = lastLocation.getTranslation();

        double sumY = 0;
        double averageY;

        for (int i = 0; i < dataY.length-1; i++)
        {
            dataY[i] = dataY[i+1];
        }

        // At dataX slot 19 input the new data point
        dataY[dataY.length - 1] =  translation.get(1) / mmPerInch;

        String s = "";

        for (int i = 0; i < dataY.length; i++)
        {
            sumY += dataY[i];
            s += "," + dataY[i];
        }


        averageY = sumY / dataY.length;

        telemetry.addData("Y Value Average: ", averageY);
        telemetry.update();

        return averageY;

    }

    double headingAverage ()
    {
        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

        double sumHeading = 0;
        double averageHeading;

        for (int i = 0; i < dataHeading.length-1; i++)
        {
            dataHeading[i] = dataHeading[i+1];
        }

        // At dataX slot 19 input the new data point
        dataHeading[dataHeading.length - 1] =  rotation.thirdAngle;

        String s = "";

        for (int i = 0; i < dataHeading.length; i++)
        {
            sumHeading += dataHeading[i];
            s += "," + dataHeading[i];
        }


        averageHeading = sumHeading / dataHeading.length;
        telemetry.addData("Heading Angle Average: ", averageHeading);
        telemetry.update();
        return averageHeading;

    }

    void runToPosition (double targetX, double targetY)
    {
        double con = 0.01;
        boolean xComplete = false;
        boolean yComplete = false;

        turn(0.4);
        sleep(1000);
        while (!xComplete) {

            xAverage();
            headingAverage();

            double errorX = targetX - xAverage();

            double xPower = (Math.abs(errorX) * con);

            telemetry.addData("xPower: ", xPower);
            telemetry.update();

            if (xPower < 0.05) {
                totalError += errorX;
                xComplete = true;
            }
            straight(xPower);

            sleep(1000);
        }

        turn(0.4);
        sleep(1000);

        while (!yComplete) {

            yAverage();
            headingAverage();

            double errorY = targetY - yAverage();

            double yPower = (Math.abs(errorY) * con);

            if (yPower < 0.05) {
                totalError += errorY;
                yComplete = true;
            }
            straight(yPower);

        }

    }

    // neg speed goes back
    void straight(double speed)
    {
        speed = -speed;
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }

    // neg speed goes right
    void strafe (double speed)
    {
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
    }

    // neg speed turns right
    void turn (double speed)
    {
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive (double speed, double distance, double angle)
    {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {



            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * CLICKS_PER_INCH);
            newLeftTarget = leftFront.getCurrentPosition() + moveCounts;
            newRightTarget = rightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftFront.setPower(speed);
            rightFront.setPower(speed);

            // Adjust angle to be positive 0 - 360
            if (angle < 0)
                angle += 360;

            // keep looping and adjusting steer angle while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                Range.clip(steer, -TURN_SPEED, TURN_SPEED);
                leftSpeed = steer;
                rightSpeed = -steer;

                leftBack.setPower(leftSpeed);
                rightBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.addData("Steer Speed",   "%5.2f, %5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle, double maxTimeMS)
    {

        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        if (angle < 0)
            angle += 360;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && (turnTimer.milliseconds() < maxTimeMS)) {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.addData(">:", "Gyro Turn is Looping");
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
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
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
        }

        // Send desired speeds to motors.
        leftSpeed = steer;
        rightSpeed = -steer;
        leftFront.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);
        leftBack.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);

        // Display it for the driver.
        //telemetry.addData("Target", "%5.2f", angle);
        //telemetry.addData("Err/St", "%5.2f / %5.2f", error, steer);
       // telemetry.addData("Speed Steer.", "%5.2f, %5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

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
        currentAngle = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        if (currentAngle < 0)
            currentAngle += 360;

        clockwiseAngle = currentAngle - targetAngle;
        if (clockwiseAngle < 0)
            clockwiseAngle += 360;

        counterClockwiseAngle = targetAngle - currentAngle;
        if (counterClockwiseAngle < 0)
            counterClockwiseAngle += 360;

       /* telemetry.addData("Target Angle: ", "%5.2f", targetAngle);
        telemetry.addData("Current Angle: ", "%5.2f", currentAngle);
        telemetry.addData("CW Angle: ", "%5.2f", clockwiseAngle);
        telemetry.addData("CCW Angle: ", "%5.2f", counterClockwiseAngle);
        telemetry.update();

        */
        leftBack.setPower(0);
        rightBack.setPower(0);
        //sleep(100);

        if (counterClockwiseAngle <= clockwiseAngle)
            return counterClockwiseAngle;               // Turn Left (positive)
        else
            return -clockwiseAngle;                     // Turn Right (negative)
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        double steer = error * PCoeff;
        if (steer < 0.0 && steer > -MIN_TURN_SPEED)
            steer = -MIN_TURN_SPEED;
        else if (steer > 0.0 && steer < MIN_TURN_SPEED)
            steer = MIN_TURN_SPEED;
        return Range.clip(error * PCoeff, -TURN_SPEED, TURN_SPEED);
    }

    public double findAngle(double x2, double y2)
    {
        // Assumes that you are pointing in flat plan on x
        double radiansDesired;
        double degreesDesired;
        double x1 = xAverage();
        double y1 = yAverage();

        double currentAngle = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;

        //Converts current Angle to radians
        //currentAngle = currentAngle/(Math.PI/180);

        if (x2 < xAverage())
        {
            radiansDesired = Math.atan((y2 - y1) / (x2 - x1));

            degreesDesired = ((radiansDesired * 180)/Math.PI);

            degreesDesired += 180;

        } else {
            radiansDesired = Math.atan((y2 - y1) / (x2 - x1));
            degreesDesired = ((radiansDesired * 180)/Math.PI);

        }

        if (degreesDesired > 180)
        {
            degreesDesired -= 360;
        }

        //degreesDesired = ((radiansDesired * 180)/Math.PI);


      //  while (degreesDesired >= 360)
       // {
       //     degreesDesired -= 360;
       // }
        degreesDesired -= currentAngle;

        return degreesDesired;

    }
    public double findDistance (double x, double y)
    {
        double currentX = xAverage();
        double currentY = yAverage();

        // uses the basic Distance Formula
        double distance = (Math.sqrt((Math.pow(x - currentX, 2) + Math.pow((y - currentY), 2))));

        return distance;
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTargetFront;
        int newRightTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;
        final double     COUNTS_PER_MOTOR_REV    = 1497.325 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);
        leftInches = -leftInches;
        rightInches = -rightInches;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetFront = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetFront = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTargetBack = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetBack = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTargetFront);
            rightFront.setTargetPosition(newRightTargetFront);
            leftBack.setTargetPosition(newLeftTargetBack);
            rightBack.setTargetPosition(newRightTargetBack);


            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTargetFront,  newRightTargetFront);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    
    public void driveToPoint (double x, double y, boolean yFirst)
    {
        gyroTurn(1, 0, 2000);
        double dX = x - xAverage();
        double yD = y - yAverage();



        if (yFirst)
        {
            if (yD > 0)
            {
                gyroTurn(1, 90, 2000);
            }
            else
            {
                gyroTurn(1, -90, 2000);
            }
            encoderDrive( 0.4, Math.abs(yD), Math.abs(yD), 2000);
            if (dX > 0)
            {
                gyroTurn(1, 0, 2000);
            }
            else
            {
                gyroTurn(1, 180, 2000);
            }
            encoderDrive(0.4, Math.abs(dX), Math.abs(dX), 2000);
        }
        else
        {
            if (dX > 0)
            {
                gyroTurn(1, 0, 2000);
            }
            else
            {
                gyroTurn(1, 180, 2000);
            }
            encoderDrive(0.4, dX, dX, 2000);
            if (yD > 0)
            {
                gyroTurn(1, 90, 2000);
            }
            else
            {
                gyroTurn(1, -90, 2000);
            }
            encoderDrive( 0.4, yD, yD, 2000);
        }


    }
}



