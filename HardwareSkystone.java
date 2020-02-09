package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class HardwareSkystone
{
    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    public final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    public final double SCALE_FACTOR = 255;


    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;

    public static final String VUFORIA_KEY =
            "AXX6Dh7/////AAABmbgZiIyG1E9ThQ6TXfPkd8M6i6PCEXDZjkt2SpXyFdQmm+CuRQp" +
                    "6A9osXLHPk1eusykK9vUsTv4XVG+T9Ikgor/aU0YALMCsVj6t0igpi5T31wEak6" +
                    "wMpvaJTSS1uEJ99G99yLh6TsHIMHBRICuGERoSBfnRwowu2/FYIv8eFsN52R++XyF" +
                    "nC8DGk/XIZSeWb6v5adayCKLUaUSBXeuTAZLHj17j0TJojO3osifAfe2JmHwKkFYozZ" +
                    "kkzt+eRkImzOFPkcnE3D2otxVbbPNqUtns0lo0Dh7Ifd0p+M4MfWeWXHf/kN19Q7pIHqf3ie" +
                    "nh58D/8Y+zsoSHtoc/62SgM+K+oVhscYHu/DpIVAFo0IXA";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField = 36 * mmPerInch;

    // Class Members
    public OpenGLMatrix lastLocation = null;

    // Declare Drive Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;

    // Declare Other Motors
    public DcMotor liftMotor = null;
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;

    // Declare Servos
    public Servo hook1 = null;
    public Servo hook2 = null;
    public Servo rotateServo = null;
    public Servo pinchServo = null;
    public Servo rotateEndServo = null;
    public Servo clampServo = null;
    public Servo clampCapstoneServo = null;
    public Servo rotateCapstoneServo = null;
    //public Servo intake1Servo = null;
    public Servo intakeServo = null;
    public Servo rotateIntakeServo = null;

    // Declare All Sensors
    BNO055IMU imu;  // IMU sensor included in Rev hub
    //ColorSensor sensorColor;
    DistanceSensor distanceSensorLeft;
    DistanceSensor distanceSensorRight;
    DistanceSensor sensorRange;
    DigitalChannel btn;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;


    /* Constructor */
    public HardwareSkystone()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        initializeRobot();
    }

    // Private Methods
    private void initializeRobot()
    {
        initDriveMotors();
        initOtherMotors();
        initServos();
        initGyro();
        initSensors();
    }

    private void initSensors()
    {
        // get a reference to the distance sensor that shares the same name.
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        sensorRange = hardwareMap.get(DistanceSensor.class, "range");

        btn = hardwareMap.get(DigitalChannel.class, "btn");
    }

    private void initGyro()
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
        parametersG.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersG.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersG.loggingEnabled = true;
        parametersG.loggingTag = "IMU";
        parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersG);
    }

    public void resetGyro()
     {
        imu.close();
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
        parametersG.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersG.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersG.loggingEnabled = true;
        parametersG.loggingTag = "IMU";
        parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersG);
    }

    private void initServos()
    {
        pinchServo = hardwareMap.get(Servo.class, "pinch");
        rotateEndServo = hardwareMap.get(Servo.class, "endRotate");
        rotateServo = hardwareMap.get(Servo.class, "rotate");
        clampServo = hardwareMap.get(Servo.class, "clamp");
        clampCapstoneServo = hardwareMap.get(Servo.class, "clampCapstone");
        rotateCapstoneServo = hardwareMap.get(Servo.class, "rotateCapstone");
        hook1 = hardwareMap.get(Servo.class, "hook1");
        hook2 = hardwareMap.get(Servo.class, "hook2");
        //intake1Servo = hardwareMap.get(Servo.class, "in1");
        intakeServo = hardwareMap.get(Servo.class, "in");
        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateInArm");
    }

    private void initOtherMotors()
    {
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initDriveMotors()
    {
        // Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}