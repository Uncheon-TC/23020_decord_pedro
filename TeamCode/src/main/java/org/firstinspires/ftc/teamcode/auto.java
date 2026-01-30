package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "auto", group = "2024-2025")
public class auto extends LinearOpMode {

    private DcMotor GT;
    private DcMotorEx SL, SR;
    private DcMotor FL, FR, BL, BR;

    private Servo servo_L, servo_R;
    private IMU imu;

    private ElapsedTime timer = new ElapsedTime();

    public static double vel_off = 0.83;

    @Override
    public void runOpMode() {

        // ===== 모터 매핑 =====
        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        GT = hardwareMap.dcMotor.get("GT");

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.FORWARD);
        GT.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        GT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        flywheel_p, flywheel_i, flywheel_d, flywheel_f
                )
        );

        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        ));

        telemetry.addLine("Ready for AUTO");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            double t = timer.seconds();

            // 0초
            if (t >= 0) {
                SL.setVelocity(2400 * vel_off);
                SR.setPower(SL.getPower());
            }

            // 4초
            if (t >= 4) servo_L.setPosition(0);

            // 7초
            if (t >= 7)
                servo_R.setPosition(0.9);



            // 9초
            if (t >= 9) {
                servo_L.setPosition(0.35);
                servo_R.setPosition(0.4);

            }

            if (t >= 10)
                GT.setPower(0.6);
            // 11초
            if (t >= 12)
                servo_L.setPosition(0);


            if (t >= 14)

                servo_R.setPosition(0.9);


            // 11~12초: 전진
            if (t >= 16 && t < 17) {
                FL.setPower(0.4);
                FR.setPower(0.4);
                BL.setPower(0.4);
                BR.setPower(0.4);
            }

            // 종료
            if (t >= 17) {
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
                SL.setPower(0);
                SR.setPower(0);
                GT.setPower(0);
                break;
            }

            telemetry.addData("AUTO Time", t);
            telemetry.update();
        }
    }
}
