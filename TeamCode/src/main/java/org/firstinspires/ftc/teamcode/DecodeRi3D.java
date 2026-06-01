/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static java.lang.Thread.sleep;
import java.util.List;
import java.util.concurrent.TimeUnit;
/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(name = "DECODE Ri3D", group = "StarterBot")
//@Disabled
public class DecodeRi3D extends OpMode {

    public final double DESIRED_DISTANCE = 36; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public final double SPEED_GAIN = 0.1;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN = 0.075;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public final double TURN_GAIN = 0.05;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    public final double MAX_AUTO_TURN = 0.3;

    private Follower follower;
    final double FEED_TIME_SECONDS = 2; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = -0.5;

    // final double WAIT_TIME_SECONDS = 5; // used in wait case to give extra time to get it to full speed


    public final double LAUNCHER_CLOSE_TARGET_VELOCITY = -470; //in ticks/second for the close goal.
    public final double LAUNCHER_CLOSE_MIN_VELOCITY = 455; //minimum required to start a shot for close goal.

    public final double LAUNCHER_FAR_TARGET_VELOCITY =-603 ; //Target velocity for far goal
    public final double LAUNCHER_FAR_MIN_VELOCITY = -680; //minimum required to start a shot for far goal.
    //-1437
    //-1307
    public final double LAUNCHER_MEDIUM_TARGET_VELOCITY = -530; //in ticks/second for the close goal.
    public final double LAUNCHER_MEDIUM_MIN_VELOCITY = -515;

    public double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    public double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    public double range = 0;
    public final double LEFT_POSITION = 0; //the left and right position for the diverter servo
    public final double RIGHT_POSITION = 1;

    // Declare OpMode members.
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotorEx leftLauncher = null;
    public DcMotorEx rightLauncher = null;
    public DcMotor intake = null;
    public CRServo leftFeeder = null;
    public CRServo rightFeeder = null;
    public Servo diverter = null;
    public Servo Led = null;

    public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int BDESIRED_TAG_ID = 20;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int RDESIRED_TAG_ID = 24;

    public boolean shotRequested = false;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();
    // ElapsedTime WaitTimer = new ElapsedTime();

    public enum LaunchState {
     IDLE,
     SPIN_UP,
     LAUNCH_RIGHT,
     LAUNCH_LEFT,
     INTAKE,
     STOP
    }

    private LaunchState launchState;



    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }

    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;





    // Setup a variable for each drive wheel to save power level for telemetry
    public double leftFrontPower;
    public double rightFrontPower;
    public double leftBackPower;
    public double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the Apriltag Detection process
        initAprilTag();

        //PEDRO init
        follower = Constants.createFollower(hardwareMap);


        launchState = LaunchState.IDLE;


        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "launcherServo");
        rightFeeder = hardwareMap.get(CRServo.class, "launcherServo2");
        diverter = hardwareMap.get(Servo.class, "diverter");
        Led = hardwareMap.get(Servo.class,"Led");
        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);


        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorBackRight.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        //PEDRO telemetry

//        Drawing.drawPoseHistory(follower.getPoseHistory());
//        Drawing.drawRobot(follower.getPose());
//        Drawing.sendPacket();

        boolean targetFound = false;
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;

        desiredTag = null;


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((BDESIRED_TAG_ID < 0) || (detection.id == BDESIRED_TAG_ID || detection.id == RDESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.


        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad2.left_trigger > 0 && targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.


            range = (desiredTag.ftcPose.range);
            // double rangeError = 0;
            double headingError = desiredTag.ftcPose.bearing;
            // double headingError = 0;
            // double rangeError = (desiredTag.ftcPose.y - DESIRED_DISTANCE);
            //double headingError = desiredTag.ftcPose.x;
            //double yawError = 0;
            //double yawError = desiredTag.ftcPose.yaw;THIS ONE

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            //drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            //turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            moveRobot(-strafe);
            //telemetry.addData("drive strafe yaw","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // telemetry.addData("range error",desiredTag.ftcPose.range - DESIRED_DISTANCE);
            //telemetry.addData("heading error",desiredTag.ftcPose.bearing );
            telemetry.addData("yaw error", desiredTag.ftcPose.yaw);
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.update();

                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);


              if (range < 104 && range > 55) {
                launcherTarget = LAUNCHER_MEDIUM_TARGET_VELOCITY;
                launcherMin = LAUNCHER_MEDIUM_MIN_VELOCITY;
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);


            } else if (range < 55) {
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);


            } else {
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;

            }


//


//            while ((leftLauncher.getVelocity() < launcherMin || leftLauncher.getVelocity() > launcherTarget) && gamepad2.a ) {
//                telemetry.addData("Left Launcher Velocity", leftLauncher.getVelocity());
//                telemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
//                telemetry.update();
//            }
//        intakeOn();
//        leftFeeder.setPower(1);
//        rightFeeder.setPower(1);

        }


        mecanumDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (Math.abs(leftLauncher.getVelocity()) < Math.abs(launcherMin)) {
            //Led.setPosition(.1900);
            Led.close();
            Led.setPosition(.1930);
        }

        if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            intake.setPower(1);

        } else if (gamepad1.left_trigger > 0 || gamepad2.y) {
            intake.setPower(-1);

        } else {
            intake.setPower(0);
        }

        if (gamepad2.b) { // stop flywheel
            leftLauncher.setVelocity(STOP_SPEED);
            rightLauncher.setVelocity(STOP_SPEED);
        }

        if (leftLauncher.getVelocity() > 200) {
            if (!gamepad2.left_bumper) {
                leftFeeder.setPower(-1);
                rightFeeder.setPower(-1);
            }
            }

            if (gamepad2.left_bumper) {
                if (getRuntime() > 10) {
                    resetRuntime();
                }
                rightFeeder.setPower(1);
                if (getRuntime() > .5) {
                    leftFeeder.setPower(1);

                }
            } else {

                rightFeeder.setPower(-1);
                leftFeeder.setPower(-1);

            }

            if (gamepad2.x) {
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
            }

            /*
             * Now we call our "Launch" function.
             */


            telemetry.addData("Left Launcher Velocity", leftLauncher.getVelocity());
            telemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
            telemetry.addData("target velocity", launcherTarget);
            telemetry.addData("minimum target velocity", launcherMin);
            telemetry.addData("diverter direction", diverterDirection);


            telemetry.update();


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    Led.setPosition(0);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        motorFrontLeft.setPower(leftFrontPower);
        motorFrontRight.setPower(rightFrontPower);
        motorBackLeft.setPower(leftBackPower);
        motorBackRight.setPower(rightBackPower);

    }


    public void stopFeeders (){

        rightFeeder.setPower(0);
        leftFeeder.setPower(0);
    }
    public void stopLaunch (){

        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
        leftFeeder.setPower(0);
    }


    public void intakeOn(){
        intake.setPower(1);
    }
    public void intakeOff(){
        intake.setPower(0);
    }
    public void expel(){
        leftLauncher.setVelocity(1000);
        rightLauncher.setVelocity(1000);
    }
    public void launchThree(){
        double shots = 0;
        switch (launchState) {
            case IDLE:
                launchState = LaunchState.SPIN_UP;
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(LAUNCHER_FAR_TARGET_VELOCITY);
                rightLauncher.setVelocity(-LAUNCHER_FAR_TARGET_VELOCITY);
                if (gamepad2.a) {
                    while ((leftLauncher.getVelocity() < LAUNCHER_FAR_MIN_VELOCITY || leftLauncher.getVelocity() > LAUNCHER_FAR_TARGET_VELOCITY) && gamepad2.a ) {
                        leftLauncher.setVelocity(LAUNCHER_FAR_TARGET_VELOCITY);
                        rightLauncher.setVelocity(-LAUNCHER_FAR_TARGET_VELOCITY);
                    }
                    shots += 1;
                    if (shots == 1) {
                        launchState = LaunchState.LAUNCH_RIGHT;
                    } else if (shots == 2) {
                        launchState = LaunchState.LAUNCH_LEFT;
                    } else if (shots == 3) {
                        launchState = LaunchState.INTAKE;
                    }
                }
                break;
            case LAUNCH_RIGHT:
                rightFeeder.setPower(FULL_SPEED);
                launchState = LaunchState.SPIN_UP;
                break;
            case LAUNCH_LEFT:
                leftFeeder.setPower(FULL_SPEED);
                launchState = LaunchState.SPIN_UP;
                break;
            case INTAKE:
                intakeOn();
                launchState = LaunchState.STOP;
                break;
            case STOP:
                intakeOff();
                stopFeeders();
                stopLaunch();
                launchState = LaunchState.IDLE;
        }

    }
    public void moveRobot(double bering) {

        double denominator = Math.max(Math.abs(bering), 1);



        /*
I made it just yaw because the only thing I wanted it to do was turn.
to turn it back just take the driveing function and make forward=y and strafe=x
and turn=yaw.
then you can uncomment the lines near the top setting the Error varibles labed THIS ONE
and the range.clip functions on top of the turn one
the final thing you need to is make it so that when you call the function it has the
correct amount of varibles
 */


        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
        intake.setPower(0);

        leftFrontPower = (bering) / denominator;
        rightFrontPower = (-bering) / denominator;
        leftBackPower = (bering) / denominator;
        rightBackPower = (-bering) / denominator;

        motorFrontLeft.setPower(leftFrontPower * 1.5);
        motorFrontRight.setPower(rightFrontPower * 1.5);
        motorBackLeft.setPower(leftBackPower * 1.5);
        motorBackRight.setPower(rightBackPower * 1.5);


        telemetry.addData("front left power", leftFrontPower);
        telemetry.addData("front right power", -rightFrontPower);
        telemetry.addData("back left power", -leftBackPower);
        telemetry.addData("back right power", rightBackPower);
        telemetry.update();
    }


    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }


    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be move, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING ) {
                //sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (1 == 1) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                //sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            //sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            //sleep(20);
        }

    }
}


