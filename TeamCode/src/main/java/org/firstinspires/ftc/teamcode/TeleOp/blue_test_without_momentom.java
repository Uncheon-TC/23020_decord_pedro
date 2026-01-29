/*package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_i;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_d;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_f;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_p;
import static java.lang.Math.round;

import androidx.xr.runtime.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo; // 서보 임포트 확인
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;
import org.firstinspires.ftc.teamcode.config.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "decode 23020_BLUE", group = "2024-2025 Test OP")
public class blue_test_without_momentom extends LinearOpMode {

    private DcMotor eat, SL, SR, SA;
    private Servo servo_S; // 서보 변수
    private IMU imu;
    Turret_Tracking tracking = new Turret_Tracking();
    private Follower follower;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private static double p = 0, i = 0, d = 0;
    private double sta_p = shooter_p, sta_i = shooter_i, sta_d = shooter_d, sta_f = shooter_f;

    private PIDFController controller;
    private PIDFCoefficients pidfCoefficients;
    private double StaticTargetPosTicks;

    GoBildaPinpointDriver odo;
    private double lastP, lastI, lastD, lastF;

    // --- [추가] 서보 토글을 위한 상태 변수 ---
    private boolean lastRightBumper = false;
    private boolean isServoUp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // 하드웨어 초기화
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        pidfCoefficients = new PIDFCoefficients(p, i, d, sta_f);
        controller = new PIDFController(pidfCoefficients);

        eat = hardwareMap.dcMotor.get("eat");
        SL = hardwareMap.dcMotor.get("SL");
        SR = hardwareMap.dcMotor.get("SR");
        SA = hardwareMap.dcMotor.get("SA");
        servo_S = hardwareMap.get(Servo.class, "servo_S"); // 하드웨어 맵 확인

        eat.setDirection(DcMotorSimple.Direction.FORWARD);
        SL.setDirection(DcMotorSimple.Direction.FORWARD);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.recalibrateIMU();

        // 시작 시 서보 위치 초기화
        servo_S.setPosition(0.0);

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            follower.update();

            // PID 계수 실시간 업데이트 로직
            if (p != lastP || i != lastI || d != lastD || sta_f != lastF) {
                PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, sta_f);
                controller.setCoefficients(coeffs);
                lastP = p; lastI = i; lastD = d; lastF = sta_f;
            }

            Pose current_robot_pos = follower.getPose();

            // 드라이브 로직 (기본 이동)
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) {
                odo.recalibrateIMU();
            }

            double botHeading_pin = follower.getHeading();
            double rotX = x * Math.cos(-botHeading_pin) - y * Math.sin(-botHeading_pin);
            double rotY = x * Math.sin(-botHeading_pin) + y * Math.cos(-botHeading_pin);

            // --- [추가] 오른쪽 범퍼 서보 토글 로직 ---
            if (gamepad1.right_bumper && !lastRightBumper) {
                if (isServoUp) {
                    servo_S.setPosition(0.0); // 내려가는 위치
                } else {
                    servo_S.setPosition(0.5); // 올라가는 위치 (필요시 0.5를 다른 값으로 수정)
                }
                isServoUp = !isServoUp;
            }
            lastRightBumper = gamepad1.right_bumper;
            // ------------------------------------

            // 타겟 트래킹 (RED로 수정된 부분 유지)
            StaticTargetPosTicks = tracking.fix_to_goal_RED(current_robot_pos);

            // 텔레메트리 출력
            telemetry.addData("Servo State", isServoUp ? "UP" : "DOWN");
            telemetry.addData("Servo Pos", servo_S.getPosition());
            telemetry.addData("target encoder", StaticTargetPosTicks);
            telemetry.addData("current encoder", SA.getCurrentPosition());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }
}*/