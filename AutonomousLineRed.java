package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "AutoLineRed", group = "Linear Opmode")
public class AutonomousLineRed extends SkyStone
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


        sleep(100);



        telemetry.addData(">:", "Ready For Start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)//////////////////
        /////////////////////////////////////////////////////////////////////
        waitForStart();

        autoArmIntakeHome();
        armHome();
        capstoneHome();

        strafeEncoder(0.5, -24, 1);

        //strafeForTime(0.4, 1.3, true);

        //gyroTurn(0.5, 0, 250);

        runtime.reset();
        while (opModeIsActive())
        {
            while (runtime.seconds() < 2)
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
                            /*
                            double object1Pos = updatedRecognitions.get(0).getRight();
                            double object2Pos = updatedRecognitions.get(1).getRight();

                            String object1 = updatedRecognitions.get(0).getLabel();
                            String object2 = updatedRecognitions.get(1).getLabel();


                            //debugging
                       telemetry.addData("IN ", "IF");
                        telemetry.addData("NUM OF CUBES: ", updatedRecognitions.size());

                        telemetry.addData("Object1 :", object1);
                        telemetry.addData("Object1 Pos: ", object1Pos);
                        telemetry.addData("Object2 : ", object2);
                        telemetry.addData("Object2 Pos: ", object2Pos);

                        telemetry.update();
                        sleep(2000);




                            if (object1.equals("Stone") && object2.equals("Stone"))
                            {
                                cubePos = "Left";
                                break;
                            }


                            if (object1Pos < object2Pos)
                            {
                                if (object1.equals("Skystone"))
                                {
                                    cubePos = "Middle";
                                    break;
                                }
                            } else if (object1Pos > object2Pos)
                            {
                                if (object1.equals("Skystone"))
                                {
                                    cubePos = "Right";
                                    break;
                                }
                            } else
                            {
                                cubePos = "Middle";
                                break;
                            }

                        */

                            double object1Pos = updatedRecognitions.get(0).getLeft();
                            double object2Pos = updatedRecognitions.get(1).getLeft();

                            String object1 = updatedRecognitions.get(0).getLabel();
                            String object2 = updatedRecognitions.get(1).getLabel();

                            /*
                            //debugging
                            telemetry.addData("IN ", "IF");
                            telemetry.addData("NUM OF CUBES: ", updatedRecognitions.size());

                            telemetry.addData("Object1 :", object1);
                            telemetry.addData("Object1 Pos: ", object1Pos);
                            telemetry.addData("Object2 : ", object2);
                            telemetry.addData("Object2 Pos: ", object2Pos);

                            telemetry.update();
                            sleep(5000);

                             */

                            //For RED SIDE ONLY//
                            //both cubes are stones
                            if (object1.equals("Stone") && object2.equals("Stone"))
                            {
                                cubePos = "Left";
                                break;
                            }
                            //the left most cube in sight is a skystone
                            else if (object1Pos > object2Pos && object1.equals("Skystone"))
                            {
                                cubePos = "Right";
                                break;
                            }
                            //
                            else if (object1Pos < object2Pos && object1.equals("Skystone"))
                            {
                                cubePos = "Middle";
                                break;
                            }
                            else if(object1Pos < object2Pos && object2.equals("Skystone"))
                            {
                                cubePos = "Right";
                                break;
                            }
                            else if (object1Pos > object2Pos && object2.equals("Skystone"))
                            {
                                cubePos = "Middle";
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
                //sleep(3000);

                redLineStartLeft();


                driveEncoderPID(5);

                gyroTurn(0.7, 75, 1000);

                driveEncoderPID(-10);

                hooksDown();

                sleep(700);

                driveEncoderPID(35);

                gyroTurn(1, 0, 2000);

                hooksUp();

                gyroTurn(0.5, 340, 500);
                driveEncoderPID(35);


            }
            else if (cubePos.equals("Middle"))
            {

                // Middle Case
                telemetry.addData("Cube Posistion:", "MIDDLE");
                telemetry.update();

                //sleep(3000);

                redLineStartMiddle();

                driveEncoderPID(5);

                gyroTurn(0.7, 75, 1000);

                driveEncoderPID(-10);

                hooksDown();

                sleep(700);

                driveEncoderPID(35);

                gyroTurn(1, 0, 2000);

                hooksUp();

                gyroTurn(0.5, 330, 500);
                driveEncoderPID(35);

            }
            else
            {

                // Right Case
                telemetry.addData("Cube Posistion:", "RIGHT");
                telemetry.update();

                //sleep(3000);

                redLineStartRight();



                driveEncoderPID(3);

                gyroTurn(0.7, 75, 1000);

                driveEncoderPID(-8);

                hooksDown();

                sleep(700);

                driveEncoderPID(35);

                gyroTurn(1, 0, 2000);

                hooksUp();

                //driveEncoderPID(-5);

                gyroTurn(0.5, 340, 500);
                driveEncoderPID(35);

            }

            sleep(200);
            break;
        }
    }
}
