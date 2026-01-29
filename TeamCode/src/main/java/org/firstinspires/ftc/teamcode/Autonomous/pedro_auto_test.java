package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "basic_pedro_test", group = "2025-2026 Test OP")
public class pedro_auto_test extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(72,72,Math.toRadians(90));
    private final Pose middlePose = new Pose(102,102,Math.toRadians(270));
    private final Pose endPose = new Pose(102, 125, Math.toRadians(180));

    private Path go1_path, go2_path;
    private PathChain chain_test;



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void loop() {



        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("path", pathState);


        panelsTelemetry.update(telemetry);

    }


    public void buildPaths() { //경로 만들기
        go1_path = new Path(new BezierLine(startPose, middlePose));
        go1_path.setLinearHeadingInterpolation(startPose.getHeading(), middlePose.getHeading());

        go2_path = new Path(new BezierLine(middlePose, endPose));
        go2_path.setLinearHeadingInterpolation(middlePose.getHeading(), endPose.getHeading());

        chain_test = follower.pathBuilder()
                .addPath(new BezierLine(endPose, middlePose))
                .setLinearHeadingInterpolation(endPose.getHeading(), middlePose.getHeading())
                .addPath(new BezierLine(middlePose, startPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {  //경로 상태 관리하기
        switch (pathState) {
            case 0:
                follower.followPath(go1_path);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(go2_path);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(chain_test);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {  //경로상태 업데이트
        pathState = pState;
        pathTimer.resetTimer();
    }


}
