package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.*;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "decode blue", group = "2024-2025 Test OP")
public class blue_teleop extends LinearOpMode {

    private DcMotor GT;
    private DcMotorEx SL, SR;
    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;

    private Servo servo_L, servo_R;
    private double shooter_power;

    public static double vel_off = 0.83;

    private ColorSensor colorSensor_L, colorSensor_R;
    private Servo light_L, light_R;

    private IMU imu;

    private Follower follower;

    private final int GREEN_DIFF = 15;
    private final int PURPLE_DIFF = 20;

    // 🔹 dpad로 조절되는 슈터 목표 속도
    private double targetMotorVelocity = 0;
    private static final double VELOCITY_STEP = 50;

    @Override
    public void runOpMode() throws InterruptedException {

        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        GT = hardwareMap.dcMotor.get("GT");

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(5,76,90));

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.FORWARD);
        GT.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        GT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients =
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        flywheel_p, flywheel_i, flywheel_d, flywheel_f
                );

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

        FrontLeftMotor = hardwareMap.dcMotor.get("FL");
        FrontRightMotor = hardwareMap.dcMotor.get("FR");
        BackLeftMotor = hardwareMap.dcMotor.get("BL");
        BackRightMotor = hardwareMap.dcMotor.get("BR");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(parameters);

        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");

        colorSensor_L = hardwareMap.get(ColorSensor.class, "colorSensor_L");
        colorSensor_R = hardwareMap.get(ColorSensor.class, "colorSensor_R");

        light_L = hardwareMap.servo.get("light_L");
        light_R = hardwareMap.servo.get("light_R");

        try { colorSensor_L.enableLed(true); } catch (Exception ignored) {}
        try { colorSensor_R.enableLed(true); } catch (Exception ignored) {}

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        boolean shooterOn = false;

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            shooter.ShotResult result =
                    shooter.calculateShot(
                            follower.getPose(),
                            BLUE_GOAL,
                            SCORE_HEIGHT,
                            follower.getVelocity(),
                            SCORE_ANGLE
                    );

            // ===== 드라이브 =====
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            FrontLeftMotor.setPower((rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower((rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower((rotY + rotX + rx) / denominator * slow);

            // ===== 슈터 ON / OFF =====
            if (rising_edge(currentGamepad1.x, previousGamepad1.x))
                shooterOn = true;
            if (rising_edge(currentGamepad1.y, previousGamepad1.y))
                shooterOn = false;

            // ===== dpad로 슈터 속도 조절 =====
            if (rising_edge(currentGamepad1.dpad_up, previousGamepad1.dpad_up)) {
                targetMotorVelocity += VELOCITY_STEP;
            }

            if (rising_edge(currentGamepad1.dpad_down, previousGamepad1.dpad_down)) {
                targetMotorVelocity = Math.max(0, targetMotorVelocity - VELOCITY_STEP);
            }

            // ===== 슈터 제어 =====
            if (shooterOn) {

                // shooter 처음 켤 때만 자동 계산값
                if (targetMotorVelocity == 0) {
                    targetMotorVelocity = velocityToTicks(result.launchSpeed);
                }

                SL.setVelocity(targetMotorVelocity * vel_off);
                shooter_power = SL.getPower();
                SR.setPower(shooter_power);

            } else {
                SL.setPower(0);
                SR.setPower(0);
                targetMotorVelocity = 0;
            }

            // ===== GT =====
            if (rising_edge(currentGamepad1.a, previousGamepad1.a))
                GT.setPower(0.6);
            if (rising_edge(currentGamepad1.b, previousGamepad1.b))
                GT.setPower(0);

            // ===== 서보 =====
            servo_L.setPosition(gamepad1.left_bumper ? 0 : 0.35);
            servo_R.setPosition(gamepad1.right_bumper ? 0.9 : 0.4);

            // ===== 색 센서 =====
            String detectedColor_L = detectColor(
                    colorSensor_L.red(),
                    colorSensor_L.green(),
                    colorSensor_L.blue()
            );
            setServo(light_L, detectedColor_L);

            String detectedColor_R = detectColor(
                    colorSensor_R.red(),
                    colorSensor_R.green(),
                    colorSensor_R.blue()
            );
            setServo(light_R, detectedColor_R);

            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Target Velocity", targetMotorVelocity);
            telemetry.addData("Heading (deg)",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);
    }

    private String detectColor(int red, int green, int blue) {
        if (isGreen(red, green, blue)) return "GREEN";
        else if (isPurple(red, green, blue)) return "PURPLE";
        else return "UNKNOWN";
    }

    private void setServo(Servo servo, String detectedColor) {
        switch (detectedColor) {
            case "GREEN":
                servo.setPosition(0.5);
                break;
            case "PURPLE":
                servo.setPosition(0.722);
                break;
            default:
                servo.setPosition(0);
                break;
        }
    }

    private boolean isGreen(int red, int green, int blue) {
        return (green > red && green > blue) &&
                (green - red > GREEN_DIFF) &&
                (green - blue > GREEN_DIFF) &&
                (green - red > 100);
    }

    private boolean isPurple(int red, int green, int blue) {
        return (blue > red && blue > green) &&
                ((blue - red > PURPLE_DIFF) || (blue - green > PURPLE_DIFF));
    }

    private boolean rising_edge(boolean current, boolean previous) {
        return current && !previous;
    }

    private double velocityToTicks(double velocityInchesPerSec) {
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS;
        double revsPerSec = velocityInchesPerSec / wheelCircumference;
        return revsPerSec * FLYWHEEL_TPR;
    }
}
