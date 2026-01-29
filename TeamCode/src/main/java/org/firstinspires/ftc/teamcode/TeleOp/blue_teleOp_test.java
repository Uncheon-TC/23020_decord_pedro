package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.FLYWHEEL_TPR;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SHOOTER_ANGLE_TPR;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_d;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_f;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_i;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_p;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_d;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_f;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_i;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_p;
import static java.lang.Math.round;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;
import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.sub_const.servo_pos_const;

@Configurable

@TeleOp(name = "decode 23020_BLUE", group = "2025-2026 Test OP")
public class blue_teleOp_test extends LinearOpMode {

    private TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();



    public static double vel_off = 0.64;
    public static double turret_off = 0;

    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor; //메카넘
    private DcMotor eat, SA;
    private DcMotorEx SL, SR;
    private Servo servo_S, servo_hood;
    private IMU imu;
    Turret_Tracking tracking = new Turret_Tracking();
    private Follower follower;
    private final Pose startPose = new Pose(72,72,Math.toRadians(90));

    private PIDFController controller;
    private double target_tick;

    private PIDFCoefficients pidfCoefficients;
    private double motor_power;
    private int finalTurretAngle;

    private double targetMotorVelocity;
    private double shooter_power;

    //GoBildaPinpointDriver odo;


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);

        //follower.setStartingPose(startPose);
        follower.setStartingPose(new Pose(82, 86, 0).mirror());


        pidfCoefficients = new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f);
        controller = new PIDFController(pidfCoefficients);


        FrontLeftMotor  = hardwareMap.dcMotor.get("FL");
        FrontRightMotor = hardwareMap.dcMotor.get("FR");
        BackLeftMotor   = hardwareMap.dcMotor.get("BL");
        BackRightMotor  = hardwareMap.dcMotor.get("BR");

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");
        SR  = hardwareMap.get(DcMotorEx.class, "SR");
        SA = hardwareMap.dcMotor.get("SA");

        /*odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-12, -5);  //cm?
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);*/

        eat.setDirection(DcMotorSimple.Direction.FORWARD);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);


        //SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eat.setPower(0);


        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(servo_pos_const.servo_shoot_block); // 기본 위치

        servo_hood = hardwareMap.servo.get("servo_H");
        servo_hood.setPosition(servo_pos_const.servo_hood_min);  //기본위치 찾기


        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients
                = new com.qualcomm.robotcore.hardware
                .PIDFCoefficients(flywheel_p, flywheel_i, flywheel_d, flywheel_f);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);
        //SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

        waitForStart();

        while (opModeIsActive()) { //main loop

            follower.update(); //current robot pose update


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) {
                follower.setPose(new Pose(72, 72, follower.getHeading()));
            }

            double botHeading_pin = follower.getHeading();

            double rotX = x * Math.cos(-botHeading_pin) - y * Math.sin(-botHeading_pin);
            double rotY = x * Math.sin(-botHeading_pin) + y * Math.cos(-botHeading_pin);

            rotX *= 1.1;

            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );

            FrontLeftMotor.setPower((rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower((rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower((rotY + rotX + rx) / denominator * slow);
            //메카넘 끝






            if (gamepad1.left_bumper) {
                servo_S.setPosition(servo_pos_const.servo_shoot_go);
                eat.setPower(1);
            } else {
                servo_S.setPosition(servo_pos_const.servo_shoot_block);
                eat.setPower(0);
            }

            if (gamepad1.x) eat.setPower(1);

            if (gamepad1.y) eat.setPower(0);

            /*if (gamepad1.left_bumper) {
                servo_S.setPosition(0.35);
                eat.setPower(1);
            } else {
                servo_S.setPosition(0.5);
                eat.setPower(0);
            }*/

            if (gamepad1.dpadDownWasPressed()) vel_off -= 0.005;
            if (gamepad1.dpadUpWasPressed()) vel_off += 0.005;

            if (gamepad1.dpadRightWasPressed()) turret_off += 6;
            if (gamepad1.dpadLeftWasPressed()) turret_off -= 6;




            shooter.ShotResult result = shooter.calculateShot(follower.getPose(), BLUE_GOAL, SCORE_HEIGHT, follower.getVelocity(), SCORE_ANGLE);

            if (result != null) {

                double StaticTargetPosTicks = tracking.fix_to_goal_BLUE(follower.getPose());

                double offsetTicks = (result.turretOffset / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * (105.0/25.0);

                finalTurretAngle = (int) round(StaticTargetPosTicks/* - offsetTicks*/ - turret_off);

                double clampedAngle = Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
                double hood_servo_pos = mapAngleToServo(clampedAngle);

                servo_hood.setPosition(hood_servo_pos);

                //터렛 pid 계산
                controller.setTargetPosition(finalTurretAngle);

                double currentPos = SA.getCurrentPosition();
                controller.updatePosition(currentPos);

                motor_power = controller.run();
                SA.setPower(motor_power);


            }

            if (gamepad1.a && result != null) {
                targetMotorVelocity = velocityToTicks(result.launchSpeed);

                SL.setVelocity(targetMotorVelocity*vel_off);
                shooter_power = SL.getPower();
                SR.setPower(shooter_power);
            } else {
                SL.setVelocity(0);
                SR.setVelocity(0);
            }




            ptelemetry.addData("curVelo", SL.getVelocity());
            ptelemetry.addData("tarVelo", targetMotorVelocity);
            ptelemetry.addData("curVelo_nonoff", SL.getVelocity()/vel_off);

            ptelemetry.addData("x", follower.getPose().getX());
            ptelemetry.addData("y", follower.getPose().getY());
            ptelemetry.addData("heading", Math.toDegrees(follower.getHeading()));

            ptelemetry.addData("turret_target_angle", tracking.getTargetHeading(follower.getPose()));
            ptelemetry.addData("turret_current_angle", SA.getCurrentPosition() * 360 / (537.7 * (105.0 / 25.0)));

                    //1도 틱 = (한바퀴 틱 / 360)

            ptelemetry.update(telemetry);

        }
    }

    public boolean check_shooting_zone(Pose pose) {
        if (pose.getY() >= Math.abs(pose.getX() - 72) + 72) return true;  // Y >= |x-72| + 72
        if (pose.getY() <= -Math.abs(pose.getX() - 72) + 72) return true; // Y <= -|x-72| + 72
        return false;
    }

    // 선형 보간 함수 (Linear Interpolation)
    private double mapAngleToServo(double angleRad) {
        double slope = (HOOD_SERVO_MIN - HOOD_SERVO_MAX) / (HOOD_MIN_ANGLE - HOOD_MAX_ANGLE);
        return slope * (angleRad - HOOD_MIN_ANGLE) + HOOD_SERVO_MIN;
    }

    // 속도(in/s)를 모터 속도(Ticks/s)로 변환
    private double velocityToTicks(double velocityInchesPerSec) {
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS;
        double revsPerSec = velocityInchesPerSec / wheelCircumference;
        return revsPerSec * FLYWHEEL_TPR;
    }
}
