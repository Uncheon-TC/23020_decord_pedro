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

    Servo servo_s, servo_hood, servo_eat;
    DcMotor eat, SL, SR, SA;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double tar_servo_s = 0.5;
    public static double tar_servo_hood = 0.5;
    public static double tar_servo_eat = 0.5;
    public static int tar_eat = 0;
    public static int tar_SLR = 0;
    public static int tar_SA = 0;


    @Override
    public void init() {

        servo_s = hardwareMap.servo.get("servo_S");
        servo_hood = hardwareMap.servo.get("servo_H");
        servo_eat = hardwareMap.servo.get("servo_EAT");

        servo_s.setPosition(tar_servo_s);
        servo_hood.setPosition(tar_servo_hood);
        servo_eat.setPosition(tar_servo_eat);

        eat = hardwareMap.dcMotor.get("eat");
        SL = hardwareMap.dcMotor.get("SL");
        SR = hardwareMap.dcMotor.get("SR");
        SA = hardwareMap.dcMotor.get("SA");


        eat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();

        servo_s.setPosition(tar_servo_s);
        servo_hood.setPosition(tar_servo_hood);
        servo_eat.setPosition(tar_servo_eat);

        eat.setTargetPosition(tar_eat);
        SL.setTargetPosition(tar_SLR);
        SR.setTargetPosition(tar_SLR);
        SA.setTargetPosition(tar_SA);

        eat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        panelsTelemetry.addData("current_servo_s", servo_s.getPosition());
        panelsTelemetry.addData("target_servo_s", tar_servo_s);
        panelsTelemetry.addData("current_servo_hood", servo_hood.getPosition());
        panelsTelemetry.addData("target_servo_hood", tar_servo_hood);

        panelsTelemetry.addData("current_eat", eat.getCurrentPosition());
        panelsTelemetry.addData("target_eat", tar_eat);
        panelsTelemetry.addData("current_SL", SL.getCurrentPosition());
        panelsTelemetry.addData("target_SL", tar_SLR);
        panelsTelemetry.addData("current_SR", SR.getCurrentPosition());
        panelsTelemetry.addData("target_SR", tar_SLR);
        panelsTelemetry.addData("current_SA", SA.getCurrentPosition());
        panelsTelemetry.addData("target_SA", tar_SA);

        panelsTelemetry.update(telemetry);
    }
}
