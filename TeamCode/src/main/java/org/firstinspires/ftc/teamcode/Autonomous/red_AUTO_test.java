package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.sub_const.pos_const.*;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.*;
import static java.lang.Math.round;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;
import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.sub_const.servo_pos_const;

@Autonomous(name = "red_test_close_ver2", group = "2025-2026 Test_auto", preselectTeleOp = "decode 23020_RED")
public class red_AUTO_test extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, segmentTime;
    private int pathState;


    private DcMotor eat, SA;
    private DcMotorEx SL, SR;
    private Servo servo_S, servo_hood, servo_eat;
    Turret_Tracking tracking = new Turret_Tracking();
    private PIDFController controller;
    private PIDFCoefficients pidfCoefficients;
    private double motor_power;
    private int finalTurretAngle;
    private double targetMotorVelocity;
    private boolean segmentStarted = false;

    private double shooter_power = 0;



    private Path first_shoot, eat_to_slide, shoot_from_slide, eat1, eat2, shoot1, shoot2, open_slide, eat3, shoot3, eat4, shoot4;
    private PathChain eat_shoot1, eat_shoot2, eat_shoot3, eat_shoot4;



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        segmentTime = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(RED_CLOSE_START);

        //////////////////////////////////////////

        pidfCoefficients = new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f);
        controller = new PIDFController(pidfCoefficients);

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");
        SR  = hardwareMap.get(DcMotorEx.class, "SR");
        SA = hardwareMap.dcMotor.get("SA");

        eat.setDirection(DcMotorSimple.Direction.FORWARD);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);


        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        servo_eat = hardwareMap.servo.get("servo_EAT");
        servo_eat.setPosition(servo_pos_const.servo_eat_down);


        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients
                = new com.qualcomm.robotcore.hardware
                .PIDFCoefficients(flywheel_p, flywheel_i, flywheel_d, flywheel_f);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);
        //SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void loop() {
        follower.update();
        Pose current_robot_pos = follower.getPose();  //save to Pose
        Vector current_robot_vel = follower.getVelocity();

        autonomousPathUpdate();


        shooter.ShotResult result = shooter.calculateShot(current_robot_pos, RED_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);
        if (result != null) {

            double StaticTargetPosTicks = tracking.fix_to_goal_RED(current_robot_pos);

            double offsetTicks = (result.turretOffset / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * (105.0/25.0);

            finalTurretAngle = (int) round(StaticTargetPosTicks/* + offsetTicks*/);

            double clampedAngle = Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
            double hood_servo_pos = mapAngleToServo(clampedAngle);

            servo_hood.setPosition(hood_servo_pos);

            //터렛 pid 계산
            controller.setTargetPosition(finalTurretAngle);

            double currentPos = SA.getCurrentPosition();
            controller.updatePosition(currentPos);

            motor_power = controller.run();
            SA.setPower(motor_power);

            double targetMotorVelocity = velocityToTicks(result.launchSpeed);

            SL.setVelocity(targetMotorVelocity*0.64);
            shooter_power = SL.getPower();
            SR.setPower(shooter_power);
        }








        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("path", pathState);

        panelsTelemetry.addData("turret_target", finalTurretAngle);
        panelsTelemetry.addData("turret_current", SA.getCurrentPosition());

        panelsTelemetry.addData("target_velo", targetMotorVelocity);
        panelsTelemetry.addData("current_velo", SL.getVelocity());
        panelsTelemetry.addData("cueent_vele_nonoff", SL.getVelocity()/0.64);

        //panelsTelemetry.addData("velo")

        panelsTelemetry.update(telemetry);

    }


    public void buildPaths() { //경로 만들기

//        go1_path = new Path(new BezierLine(startPose, middlePose));
//        go1_path.setLinearHeadingInterpolation(startPose.getHeading(), middlePose.getHeading());

        first_shoot = new Path(new BezierLine(RED_CLOSE_START, RED_CLOSE_ST_SHOOT));
        first_shoot.setLinearHeadingInterpolation(RED_CLOSE_START.getHeading()
                , RED_CLOSE_ST_SHOOT.getHeading());



        eat1 = new Path(new BezierCurve(RED_CLOSE_ST_SHOOT,
                RED_CLOSE_EAT1_CP,
                RED_CLOSE_EAT1));
        eat1.setLinearHeadingInterpolation(RED_CLOSE_ST_SHOOT.getHeading()
                , RED_CLOSE_EAT1.getHeading());



        shoot1 = new Path(new BezierLine(RED_CLOSE_EAT1, RED_CLOSE_SHOOT1));
        shoot1.setLinearHeadingInterpolation(RED_CLOSE_EAT1.getHeading(), RED_CLOSE_SHOOT1.getHeading());


        eat_shoot1 = follower.pathBuilder()
                .addPath(eat1)
                .addPath(shoot1)
                .build();


        eat2 = new Path(new BezierCurve(RED_CLOSE_SHOOT1,
                RED_CLOSE_EAT2_CP,
                RED_CLOSE_EAT2));
        eat2.setLinearHeadingInterpolation(RED_CLOSE_SHOOT1.getHeading(), RED_CLOSE_EAT2.getHeading());



        open_slide = new Path(new BezierCurve(RED_CLOSE_EAT2,
                RED_CLOSE_SLIDE_OPEN_CP,
                RED_CLOSE_SLIDE_OPEN));
        open_slide.setLinearHeadingInterpolation(RED_CLOSE_EAT2.getHeading(), RED_CLOSE_SLIDE_OPEN.getHeading());



        shoot2 = new Path(new BezierLine(RED_CLOSE_SLIDE_OPEN, RED_CLOSE_SHOOT1));
        shoot2.setLinearHeadingInterpolation(RED_CLOSE_SLIDE_OPEN.getHeading(), RED_CLOSE_SHOOT1.getHeading());



        /*eat_shoot2 = follower.pathBuilder()
                .addPath(eat2)
                .addPath(open_slide)
                .addPath(shoot2)
                .build();*/



        eat_to_slide = new Path(new BezierCurve(RED_CLOSE_SHOOT1,
                        RED_CLOSE_EAT_SLIDE_CP,
                        RED_CLOSE_EAT_SLIDE));
        eat_to_slide.setLinearHeadingInterpolation(RED_CLOSE_SHOOT1.getHeading(), RED_CLOSE_EAT_SLIDE.getHeading());

        shoot_from_slide = new Path(new BezierLine(RED_CLOSE_EAT_SLIDE, RED_CLOSE_SHOOT1));
        shoot_from_slide.setLinearHeadingInterpolation(RED_CLOSE_EAT_SLIDE.getHeading(), RED_CLOSE_SHOOT1.getHeading());

        eat3 = new Path(new BezierCurve(RED_CLOSE_SHOOT1,
                RED_CLOSE_EAT3_CP,
                RED_CLOSE_EAT3));
        eat3.setLinearHeadingInterpolation(RED_CLOSE_SHOOT1.getHeading(), RED_CLOSE_EAT3.getHeading());

        shoot3 = new Path(new BezierLine(RED_CLOSE_EAT3, RED_CLOSE_SHOOT1));
        shoot3.setLinearHeadingInterpolation(RED_CLOSE_EAT3.getHeading(), RED_CLOSE_SHOOT1.getHeading());

        eat_shoot3 = follower.pathBuilder()
                .addPath(eat3)
                .addPath(shoot3)
                .build();

        eat4 = new Path(new BezierCurve(RED_CLOSE_SHOOT1,
                RED_CLOSE_EAT4_CP,
                RED_CLOSE_EAT4));
        //eat4.setLinearHeadingInterpolation(RED_CLOSE_SHOOT1.getHeading(), RED_CLOSE_EAT4.getHeading());
        eat4.setTangentHeadingInterpolation();

        shoot4 = new Path(new BezierLine(RED_CLOSE_EAT4, RED_CLOSE_SHOOT1));
        shoot4.setLinearHeadingInterpolation(RED_CLOSE_EAT4.getHeading(), RED_CLOSE_SHOOT1.getHeading());

        eat_shoot4 = follower.pathBuilder()
                .addPath(eat4)
                .addPath(shoot4)
                .build();





    }

    public void autonomousPathUpdate() {  //경로 상태 관리하기
        switch (pathState) {
            case 0: //1번경로 시작
                follower.followPath(first_shoot);
                setPathState(1);
                break;

            case 1:  //1번경로 이동중
                eatting();
                if (!follower.isBusy()) {  //1번경로 도착
                    shoot();
                    //eat_servo_up();
                    setPathState(2);
                }
                break;

            case 2: //1번경로끝 대기
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    shoot_stop();
                    setPathState(3);
                }
                break;


            case 3: //2번경로 시작
                follower.followPath(eat_shoot1);
                setPathState(4);
                break;

            case 4: //2번경로 이동중
                if (!follower.isBusy()) { //2번경로 도착
                    shoot();
                    setPathState(5);
                }
                break;

            case 5: //2번경로끝 대기
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    shoot_stop();
                    setPathState(6);
                }
                break;

            case 6: //3번경로 시작
                follower.followPath(eat2);
                setPathState(7);
                break;

            case 7: //3번경로 이동중
                if (!follower.isBusy()) {  //먹으러 도착
                    follower.followPath(open_slide); //슬라이드 열러 출발
                    setPathState(8);
                }
                break;

            case 8: //열러가는중
                if (!follower.isBusy()) { //도착하면
                    setPathState(9);
                }
                break;

            case 9: //열러가서 대기
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(10);
                }
                break;

            case 10: //쏘러 출발
                follower.followPath(shoot2);
                setPathState(11);
                break;

            case 11: //쏘러가는중
                if (!follower.isBusy()) {//도착하면
                    shoot();
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(13);
                }

            case 13: //쏘러 출발
                follower.followPath(shoot_from_slide);
                eat_servo_up();
                setPathState(14);
                break;

            case 14:  //쏘러 가는중
                if (!follower.isBusy()) { //쏘러 도착
                    eat_servo_down();
                    shoot();
                    setPathState(15);
                }
                break;

            case 15:  //사격지점 대기
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    shoot_stop();
                    setPathState(16);
                }
                break;

            case 16:
                follower.followPath(eat_shoot3);
                setPathState(17);
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.setPose(new Pose(82, 86, 0));
                    shoot();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    shoot_stop();
                    setPathState(19);
                }
                break;

            case 19:
                follower.followPath(eat4);
                eat_servo_up();
                setPathState(20);
                break;

            case 20:
                if (!follower.isBusy()) {
                    setPathState(21);
                    eat_servo_down();
                }
                break;

            case 21:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    eat_servo_up();
                    setPathState(22);
                }
                break;

            case 22:
                follower.followPath(shoot4);
                setPathState(23);
                break;


            case 23:
                if (!follower.isBusy()) {
                    follower.setPose(new Pose(82, 86, 0));
                    eat_servo_down();
                    shoot();
                    setPathState(24);
                }
                break;

            //case 21:

        }
    }

    public void setPathState(int pState) {  //경로상태 업데이트
        pathState = pState;
        pathTimer.resetTimer();
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

    private void eatting() {
        eat.setPower(1);
    }

    private void stop_eatting() {
        eat.setPower(0);
    }

    private void shoot() {
        eat.setPower(1);
        servo_S.setPosition(servo_pos_const.servo_shoot_go);
    }

    private void shoot_stop() {
        //eat.setPower(0);
        servo_S.setPosition(servo_pos_const.servo_shoot_block);
    }

    private void eat_servo_up() {
        servo_eat.setPosition(servo_pos_const.servo_eat_up);
    }

    private void eat_servo_down() {
        servo_eat.setPosition(servo_pos_const.servo_eat_down);
    }


}

