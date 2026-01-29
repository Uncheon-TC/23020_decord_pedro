package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Field;

@Configurable


@TeleOp(name = "config_motor", group = "config")
public class motor_config extends OpMode {

    Servo servo_L, servo_R;
    DcMotor GT, SL, SR;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double tar_servo_L = 0.5;
    public static double tar_servo_R = 0.5;
    public static int tar_GT = 0;
    public static int tar_SLR = 0;


    @Override
    public void init() {

        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");

        servo_L.setPosition(tar_servo_L);
        servo_R.setPosition(tar_servo_R);

        GT = hardwareMap.dcMotor.get("GT");
        SL = hardwareMap.dcMotor.get("SL");
        SR = hardwareMap.dcMotor.get("SR");


        GT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        GT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();

        servo_L.setPosition(tar_servo_L);
        servo_R.setPosition(tar_servo_R);

        GT.setTargetPosition(tar_GT);
        SL.setTargetPosition(tar_SLR);
        SR.setTargetPosition(tar_SLR);

        GT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        panelsTelemetry.addData("current_servo_L", servo_L.getPosition());
        panelsTelemetry.addData("target_servo_L", tar_servo_L);
        panelsTelemetry.addData("current_servo_R", servo_R.getPosition());
        panelsTelemetry.addData("target_servo_R", tar_servo_R);

        panelsTelemetry.addData("current_GT", GT.getCurrentPosition());
        panelsTelemetry.addData("target_GT", tar_GT);
        panelsTelemetry.addData("current_SL", SL.getCurrentPosition());
        panelsTelemetry.addData("target_SL", tar_SLR);
        panelsTelemetry.addData("current_SR", SR.getCurrentPosition());
        panelsTelemetry.addData("target_SR", tar_SLR);

        panelsTelemetry.update(telemetry);
    }
}
