package org.firstinspires.ftc.teamcode.robotconfigs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.presets.PresetControl;

import java.util.ArrayList;
import java.util.Arrays;

public class robotConfig implements Components.RobotConfig {
    @Override

    public ArrayList<Components.Actuator<?>> getActuators() {
        return new ArrayList<>(Arrays.asList(
                leftFront, leftBack, rightFront, rightBack, arm1, arm2, claw, wrist, slide1, slide2
        ));
    }
    public static Components.BotMotor leftFront = new Components.BotMotor("leftFront", DcMotorSimple.Direction.REVERSE);
    public static Components.BotMotor leftBack = new Components.BotMotor("leftBack", DcMotorSimple.Direction.REVERSE);
    public static Components.BotMotor rightFront = new Components.BotMotor("rightFront", DcMotorSimple.Direction.FORWARD);
    public static Components.BotMotor rightBack = new Components.BotMotor("rightBack", DcMotorSimple.Direction.FORWARD);
    public static Components.BotServo arm1 = new Components.BotServo("arm", Servo.Direction.REVERSE, 5.4, 270);
    public static Components.BotServo arm2 = new Components.BotServo("arm2", Servo.Direction.FORWARD, 5.4, 270);
    public static Components.BotServo claw = new Components.BotServo("claw", Servo.Direction.FORWARD, 5.4, 270);
    public static Components.BotServo wrist = new Components.BotServo("wrist", Servo.Direction.FORWARD, 245.7, 270);
    public static Components.BotMotor slide1 = new Components.BotMotor("slide1", DcMotorSimple.Direction.REVERSE)
            .setTargetBounds(() -> 630.0, () -> 0.0)
            .setErrorTol(5)
            .setDefaultMovementTimeout(3.0)
            .setControlSystems(new String[]{"PID"}, new Components.ControlSystem<Components.BotMotor>(new PresetControl.PID( 0.015, 0, 0)));
    public static Components.BotMotor slide2 = new Components.BotMotor("slide2", DcMotorSimple.Direction.FORWARD)
            .setErrorTol(5)
            .setDefaultMovementTimeout(3.0)
            .setTargetBounds(() -> 630.0, () -> 0.0)
            .setControlSystems(new String[]{"PID"}, new Components.ControlSystem<Components.BotMotor>(new PresetControl.PID( 0.015, 0, 0)));
    public static IMU imu;
    public static IMU.Parameters parameters;

    public static double TARGET_PICKUP = 0;
    public static double TARGET_LOW    = 440;
    public static double TARGET_MID    = 150;
    public static double TARGET_HIGH   = 430;
    public static double TARGET_MAX    = 630;

    public static double ARM_PICKUP = 5.4;
    public static double ARM_SCORE  = 259.2;

    public static double WRIST_PICKUP = 245.7;
    public static double WRIST_SCORE  = 54.0;


    @Override
    public void generalInit() {
        imu = Components.getHardwareMap().get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        slide1.resetEncoder();
        slide2.resetEncoder();

        leftFront.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    public void autoSpecificInit() {
        String mystring = "Hi Arick";
    }
    public void teleopSpecificInit() {
        Components.initialize(opMode, this, false, true);
        arm1.setKeyPositions(
                new String[]{"pickup", "score"},
                new double[]{ARM_PICKUP, ARM_SCORE}
        );

        wrist.setKeyPositions(
                new String[]{"pickup", "score"},
                new double[]{WRIST_PICKUP, WRIST_SCORE}
        );

        slide1.setTarget(TARGET_PICKUP);
        slide2.setTarget(TARGET_PICKUP);

        arm1.setTarget(ARM_PICKUP);
        arm2.setTarget(ARM_PICKUP);

        wrist.setTarget(WRIST_PICKUP);
    }
}

