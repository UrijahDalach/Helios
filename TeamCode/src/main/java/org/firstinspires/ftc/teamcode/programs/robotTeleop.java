package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robotconfigs.robotConfig;

public class robotTeleop extends robotConfig {
    robotConfig robot = new robotConfig();

    public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);
            waitForStart();

            if (isStopRequested()) return;

            int iterations = 0;
            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y; // y stick value should be reversed
                double x = gamepad1.left_stick_x;
                double rx = -gamepad1.right_stick_x;

                // Will reset IMU in case of robot errors
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                double liftSpeed = 250; // ticks per second

                if ((useTimer==1) && (timer.seconds() > 0.7)){
                    target = 0;
                    useTimer = 0;
                }

                if (gamepad1.left_trigger>0.5) { // Checks for left trigger input, slows all motors by 50%
                    frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                    backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                    frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                    backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
                }
                if (gamepad1.a){ // Reset to pickup position
                    //linearSlidesTarget=0;
                    //SET LINEAR SLIDE TARGET DELAY
                    wristpos=0.91;
                    arm1pos=0.02;
                    arm2pos=0.02;
                    timer.reset(); //start counting time
                    useTimer = 1;


                }else if (gamepad1.b){ // Low junction
                    target=440; //rough estimate - we have to tune the encoder positions

                }else if (gamepad1.x){ // Mid junction
                    target = 150;// rough estimate - this has to be changed, the reason this is lower than low junction is because we get the added distance from the flip
                    arm1pos = 0.96;
                    arm2pos=0.96;
                    wristpos=0.2;

                }else if (gamepad1.y){ // High junction
                    target = 430; //tune encoders
                    // Add code to flip servos
                    arm1pos = 0.96;
                    arm2pos=0.96;
                    wristpos=0.2;
                }
                if (gamepad1.left_bumper){
                    //slide1.setPower(0.75);
                    //slide2.setPower(0.75);

                    if (target<630){
                        target += 15;
                    }

                }else if (gamepad1.right_bumper){
                    //slide1.setPower(-0.5);
                    //slide2.setPower(-0.5);

                    if (target > -1){
                        target -= 15;
                    }
                }

                claw.setPosition(1-gamepad1.right_trigger);
                arm1.setPosition(arm1pos);
                arm2.setPosition(arm2pos);
                wrist.setPosition((wristpos));


                int error1 = (int) (target - slide1.getCurrentPosition());
                slide1.setPower(-error1 * Kp);
                int error2 = (int) (target - slide2.getCurrentPosition());
                slide2.setPower(error2 * Kp);
                telemetry.addData("slide1 pos", slide1.getCurrentPosition());
                telemetry.addData("slide2 pos", slide2.getCurrentPosition());
                telemetry.update();


                // Sends data about robot to android phone
                telemetry.addData("iterations:",++iterations);
                telemetry.addData("Slide1:",slide1.getCurrentPosition());
                telemetry.addData("Slide2:",slide2.getCurrentPosition());
                telemetry.addData("Target:",target);
                telemetry.addData("Error1:",error1);
                telemetry.addData("Error2:",error2);
                telemetry.update();

                // Set powers for driving
                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);
            }
        }
    }
}
