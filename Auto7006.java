package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name = "Auto7006", group = "Pushbot")
public class Auto7006 extends LinearOpMode {
    OpenGLMatrix lastLocation = null;
    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z
    int VuforiaPosition = 0;
    int ShouldIReadVuforia = 1;
    RoboTitans robot = new RoboTitans();
    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .5;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.2;

    static final double HEADING_THRESHOLD = 1;
    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;

    ColorSensor colorSensor;
    Integer Blue = 0;
    Integer Red = 0;
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        float hsvValues[] = {0F, 0F, 0F};
        parameters.vuforiaLicenseKey = "Af67IXL/////AAAAGd40XPnXwElsrdW5hZYSwWxmJ9uMHusOIjRh2UVOSfHUNUsZLa0uxgyNwDW9A49V6QK3RfXroe1vCXYWpSyUAlvOfX2IviDW5in5RFgSwUsLvFxFYtlDzS6W1YS/WyzvjznMRsSNb92S4u0+rFIYuLMNkeRS3YWPIzzImaB96Cbym2rrUBUMhWqKkMwIj4eJK415xtoY+CFMopq2JT8Tcb2nO0Hv0gwTBBpw//Edd+Ghv4bkqlp/x+Uue4wV+e5zFTWSkdnYmxArM4WnmN1iCkYxkB3H4BXL4fhlGmkpI8bAcVcGsi1nTrZi7X59g9/JhbAlIOGCiXQvKZKigO/2KvOybjdFXDqNcym5vaVH6Ge1";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        final float values[] = hsvValues;
        boolean bLedOn = true;
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        // can help in debugging; otherwise not necessary
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
        telemetry.addData("Path", "Complete");
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
//************************************//SETUP IS DONE!******************************************************************************************
        int start = 0;

       // grabberClose();
        sleep(200);
        liftUP(1.0, 500);

        waitForStart();
        relicTrackables.activate();
        robot.arm.setPosition(0); //Reset robot Servo Position

        while (opModeIsActive()) {

            switch (start) {
                case 0:

//                    VuforiaCheck();
//                    if (VuforiaPosition == 1 || VuforiaPosition == 2 || VuforiaPosition == 3) {
//                        ShouldIReadVuforia = 0; //Dont read Vuforia Anymore
//                        break;
//                    }


  while (VuforiaPosition == 0 && ShouldIReadVuforia == 1) {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
//                        telemetry.addData("Pose", format(pose));
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.

                telemetry.addData("VuMark is", "Left");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);


                VuforiaPosition = 1;

                //PERFORM LEFT CODE HERE
            } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.


                telemetry.addData("VuMark is", "Center");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);

                VuforiaPosition = 2;

                //PERFORM CENTER CODE HERE
            } else if (vuMark == RelicRecoveryVuMark.RIGHT)

            { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                telemetry.addData("X =", tX);
                telemetry.addData("Y =", tY);
                telemetry.addData("Z =", tZ);


                VuforiaPosition = 3;
                //PERFORM RIGHT CODE HERE
            }

        } else

        {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();

    }


                    telemetry.addData("The VuMark reads:", VuforiaPosition);
                    telemetry.update();
                    sleep(3000);
                    start++;
                    break;
                case 1:
                     /*  Put arm down and get ready to read color */
                    robot.smackServo.setPosition(.4);
                    sleep(1000);
                    robot.arm.setPosition(0.20);
                    sleep(500);
                    robot.arm.setPosition(0.4);
                    sleep(400);
                    robot.arm.setPosition(0.6);
                    telemetry.addLine("Getting Ready to read COLOR");
                    telemetry.update();
                    sleep(500);
                    start = 2;
                    break;
                case 2:
                    /* Read colors and start approriate code for color*/
                    if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                        telemetry.addLine("Blue Detected!!");
                        telemetry.update();
                        Blue = 1;
                        sleep(1000);

                        start = 3;
                        break;

                    } else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                        telemetry.addLine("Red Detected!!");
                        telemetry.update();
                        Red = 1;
                        sleep(1000);

                        start = 4;
                        break;
                    } else { //IF NO COLOR IS DETECTED, MOVE ONTO READING VUFORIA!

                        start = 5;
                        break;
                    }
                case 3:
                    if (Blue == 1) {
                        /*Knock out red ball*/
                        telemetry.addLine("Doing Blue stuff");
                        telemetry.update();
                        robot.smackServo.setPosition(1);
                        sleep(700);
                        robot.arm.setPosition(0);
                        sleep(1500);
                        telemetry.addData("The VuMark reads:", VuforiaPosition);
                        telemetry.update();
                        start = 5;
                        break;

                    }
                case 4:
                    if (Red == 1) {
                        /* Knock out red ball*/
                        robot.smackServo.setPosition(0);
                        // DO RED STUFF
                        telemetry.addLine("Doing Red stuff");
                        telemetry.update();
                        sleep(700);
                        robot.arm.setPosition(0);
                        sleep(1500);
                        start = 5;
                        break;
                    }
                case 5:

                    if (VuforiaPosition == 1) { //Left
                        //GO STRAIGHT
                        telemetry.addLine("Running code for LEFT");
                        telemetry.update();
                        robot.left_drive.setPower(.5);
                        robot.right_drive.setPower(.5);
                        sleep(1300);

                        // gyroDrive(.5, 35, 0);

                        gyroTurn(.5, 90);
                        sleep(500);
                        gyroDrive(.10, 4, 90);
                        stop();

                        //Turn left at exact Glyph spot(Left, Center, Right)
                        //Go straight and score
                    }

                    if (VuforiaPosition == 2) { //Center
                        //GO STRAIGHT
                        telemetry.addLine("Running code for Center");
                        telemetry.update();


                        stop();

                        //Turn left at exact Glyph spot(Left, Center, Right)
                        //Go straight and score
                    }
                    if (VuforiaPosition == 3) { //Right
                        //GO STRAIGHT
                        telemetry.addLine("Running code for RIGHT");
                        telemetry.update();
                        //Turn left at exact Glyph spot(Left, Center, Right)
                        //Go straight and score

                        stop();

                    }
                    if (VuforiaPosition == -1) { //Right
                        //GO STRAIGHT
                        telemetry.addLine("STOPPING THE ROBOT!");
                        telemetry.update();
                        robot.left_drive.setPower(0);
                        robot.right_drive.setPower(0);
                        stop();

                    }
                    break;

                default:
                    telemetry.addData("Complete", "Good Job!");
                    telemetry.update();
            }
        }
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

    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        if (opModeIsActive()) {

            telemetry.update();
            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.left_drive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.right_drive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.left_drive.setTargetPosition(newLeftTarget);
            robot.right_drive.setTargetPosition(newRightTarget);

            robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.left_drive.setPower(speed);
            robot.right_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.left_drive.isBusy() && robot.right_drive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.left_drive.setPower(leftSpeed);
                robot.right_drive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.left_drive.getCurrentPosition(),
                        robot.right_drive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in pushbeacons) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.left_drive.setPower(0);
        robot.right_drive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.left_drive.setPower(leftSpeed);
        robot.right_drive.setPower(rightSpeed);

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
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
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

//    public void grabberClose() {
//        robot.leftServo.setPosition(.3); //Right Servo
//        robot.rightServo.setPosition(.65);//LEft Servo
//    }
//    public void grabberOpen() {
//        robot.leftServo.setPosition(0); //Right Servo
//        robot.rightServo.setPosition(1);//LEft Servo
//    }


    public void liftUP(Double Power, int Time) {

        robot.liftMotor.setPower(Power);
        sleep(Time);
        robot.liftMotor.setPower(0);
    }

    public void armSmackActivate() {

        robot.smackServo.setPosition(.5);
        sleep(1000);
        robot.arm.setPosition(0.30);
        sleep(500);
        robot.arm.setPosition(0.4);
        sleep(400);
        robot.arm.setPosition(0.6);
    }

    public void VuforiaCheck() {
        while (VuforiaPosition == 0 && ShouldIReadVuforia == 1) {
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
//                        telemetry.addData("Pose", format(pose));
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.

                    telemetry.addData("VuMark is", "Left");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);


                    VuforiaPosition = 1;

                    //PERFORM LEFT CODE HERE
                } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.


                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);

                    VuforiaPosition = 2;

                    //PERFORM CENTER CODE HERE
                } else if (vuMark == RelicRecoveryVuMark.RIGHT)

                { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);


                    VuforiaPosition = 3;
                    //PERFORM RIGHT CODE HERE
                }

            } else

            {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();

        }

    }

}
