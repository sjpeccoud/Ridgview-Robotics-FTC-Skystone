package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "AutoLineBlue", group = "Linear Opmode")
public class AutonomousLineBlue extends SkyStone
{

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
        capstoneInit();
        armInit();
        clampRelease();
        hooksUp();
        autoArmIntakeInit();


        sleep(100);


        //hardware.autonomousServo.setPosition(AUTONOMOUS_UP);


        telemetry.addData(">:", "Ready For Start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)//////////////////
        /////////////////////////////////////////////////////////////////////
        waitForStart();

        autoArmIntakeHome();
        armHome();
        capstoneHome();

        strafeEncoder(0.5, -24, 5);

        //strafeForTime(0.4, 1.3, true);

        //sleep(5000);
        //gyroTurn(0);

        runtime.reset();
        while (opModeIsActive())
        {
            while (runtime.seconds() < 3)
            {

                if (tfod != null)
                {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null)
                    {
                        //telemetry.addData("# Object Detected", updatedRecognitions.size());
                        //telemetry.update();
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


                            //BLUE SIDE ONLY//
                            //both cubes are stones
                            if (object1.equals("Stone") && object2.equals("Stone"))
                            {
                                cubePos = "Right";
                                break;
                            }
                            //the left most cube in sight is a skystone
                            else if (object1Pos > object2Pos && object1.equals("Skystone"))
                            {
                                cubePos = "Middle";
                                break;
                            }
                            //the left most cube in sight is a skystone
                            else if (object1Pos > object2Pos && object2.equals("Skystone"))
                            {
                                cubePos = "Left";
                                break;
                            }
                            //
                            else if (object1Pos < object2Pos && object1.equals("Skystone"))
                            {
                                cubePos = "Left";
                                break;
                            }
                            else if (object1Pos < object2Pos && object2.equals("Skystone"))
                            {
                                cubePos = "Middle";
                                break;
                            }


                        }

                    }
                }
            }

            tfod.shutdown();

            //END OF TENSORFLOW OBJECT DETECTION//


            if (cubePos.equals("Left"))
            {

                // Left Case
                telemetry.addData("Cube Posistion:", "LEFT");
                telemetry.update();


                blueLineStartLeft();



                driveEncoderPID(-10);

                gyroTurn(0.7, 100, 1000);

                driveEncoderPID(-10);

                hooksDown();

                sleep(700);

                driveEncoderPID(35);

                gyroTurn(1, 180, 2000);

                hooksUp();

                gyroTurn(0.5, 210, 500);
                driveEncoderPID(35);


            } else if (cubePos.equals("Middle"))
            {

                // Middle Case
                telemetry.addData("Cube Posistion:", "MIDDLE");
                telemetry.update();



                blueLineStartMiddle();



                driveEncoderPID(-10);

                gyroTurn(0.7, 100, 1000);

                driveEncoderPID(-10);

                hooksDown();

                sleep(700);

                driveEncoderPID(35);

                gyroTurn(1, 180, 2000);

                hooksUp();

                gyroTurn(0.5, 210, 500);
                driveEncoderPID(35);
                //gyroTurn(0.5, 270, 500);
                //strafeEncoder(1, 22, 1);
                //driveEncoderPID(20);

                //autoArmLower();


            } else
            {

                // Right Case
                telemetry.addData("Cube Posistion:", "RIGHT");
                telemetry.update();


                blueLineStartRight();


                driveEncoderPID(-15);

                gyroTurn(0.7, 100, 1000);

                driveEncoderPID(-10);

                hooksDown();

                sleep(700);

                driveEncoderPID(35);

                gyroTurn(1, 180, 2000);

                hooksUp();

                //driveEncoderPID(-5);

                gyroTurn(0.5, 210, 500);
                driveEncoderPID(35);
            }
            break;
        }
    }
}
