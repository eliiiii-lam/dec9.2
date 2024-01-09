package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import org.opencv.core.Scalar;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



import java.io.File;
@Autonomous
public class RedShortScan2 extends LinearOpMode {
   private DcMotor fl, br, fr, bl;

    private int newflTarget;
    private int newfrTarget;
    private int newblTarget;
    private int newbrTarget;

    DcMotor elbow;
    DcMotor elbow2;

    Servo clawL;
    Servo clawR;
    Servo wrist;

    private double driveSpeed = 0;

    private double turnSpeed = 0;

    private double leftSpeed = 0;
    private double rightSpeed = 0;


    static final double P_DRIVE_GAIN =  0.03;
    static final double P_TURN_GAIN = 0.02;

    static final double HEADING_THRESHOLD = 1.0;

    private double headingError = 0;
    private double targetHeading = 0;


















    BNO055IMU imu;
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    static final double WEBCAM_WIDTH = 640;
    private OpenCvCamera webcam;
    private RedContourPipeline pipeline;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3;
    private double rightBarcodeRangeBoundary = 0.7;
    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 200.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 128.0);

    //File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");


    @Override
    public void runOpMode() throws InterruptedException {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new RedContourPipeline(0.0, 0.0, 0.0, 0.0);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        fl = hardwareMap.get(DcMotor.class, "fL");
        fr = hardwareMap.get(DcMotor.class, "fR");
        br = hardwareMap.get(DcMotor.class, "bR");
        bl = hardwareMap.get(DcMotor.class, "bL");


        elbow = hardwareMap.dcMotor.get("elbow");
        elbow2 = hardwareMap.dcMotor.get("elbow2");

        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        wrist = hardwareMap.servo.get("wrist");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d : %7d: %7d ",
                fl.getCurrentPosition(),
                fr.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition());
        telemetry.update();

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive())
        {
            if(pipeline.error){
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            double rectangleArea = pipeline.getRectArea();

            telemetry.addData("Rectangle Area", rectangleArea);
            telemetry.addData("XY: ",  pipeline.getRectMidpointXY());

            if(rectangleArea > minRectangleArea){

                if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * WEBCAM_WIDTH){
                    telemetry.addData("Barcode Position", "Right");
                    sleep(2000);
                    pixelHold();
                    encoderDrive(DRIVE_SPEED,-4,0);
                    turnToHeading(TURN_SPEED, 180);
                    encoderDrive(DRIVE_SPEED,9,0);
                    holdHeading(TURN_SPEED,45,8);
                    elbowmove(DRIVE_SPEED,-40,2);
                    rightpixelRelease();
                    elbowmove(DRIVE_SPEED,45,2);
                    sleep(3000);
                    encoderDrive(DRIVE_SPEED,6,0);
                    turnToHeading(TURN_SPEED,45);
                    encoderDrive(DRIVE_SPEED, 36,0);
                    elbowmove(DRIVE_SPEED,-20,2);
                    leftpixelRelease();



                }
                else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * WEBCAM_WIDTH){
                    telemetry.addData("Barcode Position", "Left");
                    sleep(2000);
                    pixelHold();
                    encoderDrive(DRIVE_SPEED,-4,0);
                    turnToHeading(TURN_SPEED, 180);
                    encoderDrive(DRIVE_SPEED,9,0);
                    holdHeading(TURN_SPEED,-45,8);
                    elbowmove(DRIVE_SPEED,-40,2);
                    rightpixelRelease();
                    elbowmove(DRIVE_SPEED,45,2);
                    sleep(3000);
                    encoderDrive(DRIVE_SPEED,6,0);
                    turnToHeading(TURN_SPEED,45);
                    encoderDrive(DRIVE_SPEED, 36,0);
                    elbowmove(DRIVE_SPEED,-20,2);
                    leftpixelRelease();



                }
                else {
                    telemetry.addData("Barcode Position", "Center");
                    sleep(2000);
                    pixelHold();
                    encoderDrive(DRIVE_SPEED,-4,0);
                    turnToHeading(TURN_SPEED, 180);
                    encoderDrive(DRIVE_SPEED,13,0);
                    elbowmove(DRIVE_SPEED,-40,2);
                    rightpixelRelease();
                    elbowmove(DRIVE_SPEED,45,2);
                    sleep(3000);
                    encoderDrive(DRIVE_SPEED,6,0);
                    turnToHeading(TURN_SPEED,45);
                    encoderDrive(DRIVE_SPEED, 36,0);
                    elbowmove(DRIVE_SPEED,-20,2);
                    leftpixelRelease();

                }
            }
            telemetry.update();
        }
    }







    public void pixelHold() {
        clawL.setPosition(0.4);
        clawR.setPosition(0);
    }

    public void leftpixelRelease() {
        clawL.setPosition(0);
    }

    public void rightpixelRelease() {
        clawR.setPosition(0.6);
    }



    public void elbowmove(double speed,
                          double elbowenco, double timeoutS) {

        int newElbow;
        int newElbow2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newElbow = elbow.getCurrentPosition() - (int) (elbowenco * COUNTS_PER_INCH);
            newElbow2 = elbow2.getCurrentPosition() + (int) (elbowenco * COUNTS_PER_INCH);

            elbow.setTargetPosition(newElbow);
            elbow2.setTargetPosition(newElbow2);

            // Turn On RUN_TO_POSITION


            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            elbow.setPower(Math.abs(speed));
            elbow2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elbow.isBusy() && elbow2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d: %7d", newElbow, newElbow2);
                telemetry.addData("Currently at", " at %7d :%7d", newElbow, newElbow2,

                        elbow.getCurrentPosition(),
                        elbow2.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION


            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            elbow.setPower(0);
            elbow2.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }


    public void encoderDrive(double speed,
                             double distance, double heading) {



        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int moveCounts = (int) (distance * COUNTS_PER_INCH);


            // Determine new target position, and pass to motor controller
            newflTarget = fl.getCurrentPosition() + moveCounts;
            newfrTarget = fr.getCurrentPosition() + moveCounts;
            newblTarget = bl.getCurrentPosition() + moveCounts;
            newbrTarget = br.getCurrentPosition() + moveCounts;

            fl.setTargetPosition(newflTarget);
            fr.setTargetPosition(newfrTarget);
            bl.setTargetPosition(newblTarget);
            br.setTargetPosition(newbrTarget);

            // Turn On RUN_TO_POSITION
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            speed = Math.abs(speed);
            moveRobot(speed, 0);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (fr.isBusy() && br.isBusy() && bl.isBusy() && fl.isBusy())) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed);

                // Display it for the driver.
                sendTelemetry(true);
            }

            // Stop all motion;

            moveRobot(0,0);

            // Turn off RUN_TO_POSITION
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            sleep(100);

        }
    }
    public void encoderDriveStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = fl.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = bl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = fr.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = br.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            fl.setTargetPosition(newFrontLeftTarget);
            bl.setTargetPosition(newBackLeftTarget);
            fr.setTargetPosition(newFrontRightTarget);
            br.setTargetPosition(newBackRightTarget);

            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            fl.setPower(Math.abs(speed));
            bl.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        fl.getCurrentPosition(),
                        bl.getCurrentPosition(),
                        fr.getCurrentPosition(),
                        br.getCurrentPosition());
                telemetry.update();
            }

            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }
    //private double getZAngle() {return (-imu.getAngularOrientation().firstAngle);


    public void turnToHeading(double speed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)){
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0,0);
    }

    public void holdHeading(double speed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)){
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);


            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);

        }
        moveRobot(0,0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1,1);
    }


    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        leftSpeed = drive + turn;
        rightSpeed = drive - turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {   leftSpeed /= max;
            rightSpeed/= max;
        }

        fl.setPower(leftSpeed);
        bl.setPower(leftSpeed);
        fr.setPower(-rightSpeed);
        br.setPower(-rightSpeed);

    }

    private void sendTelemetry(boolean straight) {

        if (straight){
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos bL:fL:bR:fR", "%7d:%7d:%7d:%7d", bl, fl,br,fr);
            telemetry.addData("Actual Pos bL:fL:bR:fR", "%7d:%7d:%7d:%7d", bl,fl,br,fr );

        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Currecnt", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error : Steer Pwr", "%5.1f : %5.1f", headingError, TURN_SPEED);
        telemetry.addData("Wheel Speeds bL:fL:bR:fR", "%5.2f:%5.2f:%5.2f:%5.2f", bl,fl,br,fr);
        telemetry.update();


    }






    public double getHeading(){
        return (-imu.getAngularOrientation().firstAngle);
    }

}