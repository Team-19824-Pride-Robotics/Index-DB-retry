
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp
    public class RoryNyteDrive extends LinearOpMode {

    //variables
    public static double long_launch_speed = 2110;
    public static double close_launch_speed = 1650;
    double fly1Speed = 0;
    public static double intake_suck_speed = 2000;
    double intake_speed = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        // Flywheel motors
        DcMotor flyWheelLeft = hardwareMap.dcMotor.get("flyWheelLeft");
        DcMotor flyWheelRight = hardwareMap.dcMotor.get("flyWheelRight");
        // Use encoders on flywheel motors
        flyWheelLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheelRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Intake motors
        DcMotor frontIntake = hardwareMap.dcMotor.get("frontIntake");
        DcMotor rearIntake = hardwareMap.dcMotor.get("rearIntake");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(leftFrontPower);
            backLeftMotor.setPower(leftBackPower);
            frontRightMotor.setPower(rightFrontPower);
            backRightMotor.setPower(rightBackPower);

            frontLeftMotor.setPower(leftFrontPower);
            backLeftMotor.setPower(leftBackPower);
            frontRightMotor.setPower(rightFrontPower);
            backRightMotor.setPower(rightBackPower);

            // Flywheel controls that I hope work
            if (gamepad1.right_trigger > .1) {
                flyWheelRight.setPower(close_launch_speed);
                flyWheelLeft.setPower(close_launch_speed);
            } else if (gamepad1.left_trigger > .1) {
                flyWheelRight.setPower(long_launch_speed);
                flyWheelLeft.setPower(long_launch_speed);
            }
            else {
                flyWheelLeft.setPower(fly1Speed);
                flyWheelRight.setPower(fly1Speed);
            }

            // Intake controls
            if (gamepad1.a) {
                frontIntake.setPower(intake_suck_speed);
            }
            else if (gamepad1.b) {
                rearIntake.setPower(intake_suck_speed);
            }
            else {
                frontIntake.setPower(intake_speed);
                rearIntake.setPower(intake_speed);
            }
        }
    }
}

