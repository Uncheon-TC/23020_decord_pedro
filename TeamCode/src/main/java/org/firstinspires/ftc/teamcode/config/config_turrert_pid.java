package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable

@TeleOp(name = "config_turret_pid", group = "config")
public class config_turrert_pid extends OpMode {

    private TelemetryManager panelsTelemetry =  PanelsTelemetry.INSTANCE.getTelemetry();

    private PIDFController controller;
@Sorter(sort = 0)
    public static double p = 0;
@Sorter(sort = 1)
    public static double i = 0;
@Sorter(sort = 2)
    public static double d = 0;
@Sorter(sort = 3)
    public static double f = 0;
@Sorter(sort = 4)
    public static double target_deg = 0;
    public double target_tick;
    public final double ticks_per_rev = 537.7; //312 rpm
    private PIDFCoefficients pidfCoefficients;
    private double motor_power;

    private double lastP, lastI, lastD, lastF;

    DcMotor SA;

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    private ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {

        //dashboard = FtcDashboard.getInstance();
        timer.reset();

        pidfCoefficients = new PIDFCoefficients(p,i,d,f);
        controller = new PIDFController(pidfCoefficients);

        SA = hardwareMap.dcMotor.get("SA");

        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();

        if (p != lastP || i != lastI || d != lastD || f != lastF) {
            PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
            controller.setCoefficients(coeffs);

            lastP = p; lastI = i; lastD = d; lastF = f;
        }

        target_tick = (target_deg/360.0) * (105.0/25.0) * ticks_per_rev; //패널에서 목표각도 입력 -> tick으로 변환
        controller.setTargetPosition(target_tick); //pidf controller 상수 변경

        double currentPos = SA.getCurrentPosition();
        controller.updatePosition(currentPos);

        double current_deg = (currentPos) / (ticks_per_rev * (105.0/25.0)) * 360.0;


        motor_power = controller.run();

        SA.setPower(motor_power);

        //packet.put("target deg", target_deg);
        //packet.put("current deg", current_deg);

        //dashboard.sendTelemetryPacket(packet);

        double_telemetry("target deg", target_deg);
        double_telemetry("current deg", current_deg);
        double_telemetry("error deg", - target_deg + current_deg);

        double_telemetry("target tick", target_tick);
        double_telemetry("error tick", controller.getError());
        telemetry.addLine();
        double_telemetry("p", p);
        double_telemetry("i", i);
        double_telemetry("d", d);
        double_telemetry("f", f);

        double_telemetry_update();
    }

    private void double_telemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        panelsTelemetry.addData(caption, value);
    }

    private void double_telemetry(String caption, double value) {
        telemetry.addData(caption, value);
        panelsTelemetry.addData(caption, value);
    }

    private void double_telemetry_update() {
        panelsTelemetry.update(telemetry);
        telemetry.update();
    }
}

