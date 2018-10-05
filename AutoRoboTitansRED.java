package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Roshaan on 11/2/2017.
 */
@Autonomous(name="RobotTitansAutoRED", group="K9bot")

public class AutoRoboTitansRED extends LinearOpMode {
    double          armPosition     = .38;                   // Servo safe position

    RoboTitans robot   = new RoboTitans();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    final double    ARM_SPEED       = 0.05 ;

    ColorSensor colorSensor;
    Integer          Blue  = 0 ;
    Integer          Red = 0;
    @Override
    public void runOpMode() {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        robot.arm.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        //telemetry.addData(">", "Root Heading = %d", gyro.getIntegratedZValue());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        int start = 1;

        while (opModeIsActive()) {

            switch (start) {
                case 1:
                    sleep(1000);
                    robot.arm.setPosition(0.10);
                    sleep(700);
                    robot.arm.setPosition(0.45);
                    telemetry.addLine("Getting Ready to read COLOR");
                    telemetry.update();
                    sleep(1000);
                    start++;
                    break;
                case 2:
                    if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                        telemetry.addLine("Blue Detected!!");
                        telemetry.update();
                        Blue = 1;
                        telemetry.addData("Blue Detected?", Blue);

                        telemetry.addData("Red Detected?", Red);
                        telemetry.update();
                        sleep(2000);
                        start = 3;
                        break;

                    } else {
                        telemetry.addLine("Red Detected!!");
                        telemetry.update();
                        Red = 1;
                        telemetry.addData("Blue Detected?", Blue);
                        telemetry.addData("Red Detected?", Red);
                        telemetry.update();
                        sleep(2000);
                        start = 4;
                        break;
                    }
                case 3:
                    if (Blue == 1) {
                        robot.rightDrive.setPower(.7);
                        robot.leftDrive.setPower(.7);
                        sleep(1000);
                        robot.rightDrive.setPower(0);
                        robot.leftDrive.setPower(0);
                        sleep(1000);
                        telemetry.addLine("Doing Blue stuff");
                        telemetry.update();
                    }
                case 4:
                    if (Red == 1){
                        robot.rightDrive.setPower(-.7);
                        robot.leftDrive.setPower(-.7);
                        sleep(1000);
                        robot.rightDrive.setPower(0);
                        robot.leftDrive.setPower(0);
                        sleep(1000);
                        telemetry.addLine("Doing Red stuff");
                        telemetry.update();
                    }
                default:
                    telemetry.addData("Complete", "Good Job!");
                    telemetry.update();
            }
        }
    }}
