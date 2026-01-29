package org.firstinspires.ftc.teamcode.auto_cal;

import androidx.vectordrawable.graphics.drawable.VectorDrawableCompat;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class shooter {

    // 중력 가속도 (in/s^2)
    public static final double GRAVITY = 386.1;

    public static class ShotResult {
        public double hoodAngle;     // 발사 각도 (Radians)
        public double launchSpeed;   // 발사 속도 (in/s)
        public double turretOffset;  // 터렛 보정 각도 (Radians)
        public double timeOfFlight;  // 비행 시간 (Seconds)

        public ShotResult(double angle, double speed, double offset, double time) {
            this.hoodAngle = angle;
            this.launchSpeed = speed;
            this.turretOffset = offset;
            this.timeOfFlight = time;
        }
    }

    /**
     * 전체 슈팅 솔루션 계산 (정적 계산 + 동적 보정 통합)
     * @param robot  로봇 위치 (Field Coordinates)
     * @param goal    골대 위치
     * @param goalHeight      골대 높이 - 슈터 높이 차이 (inches)
     * @param robotVel        로봇 속도 (in/s, Field Centric)
     * @param targetEntryAngle 목표물 입사 각도 (Radians, 보통 음수 ex: -30도)
     */
    public static ShotResult calculateShot(
            Pose robot,
            Pose goal,
            double goalHeight,
            Vector robotVel,
            double targetEntryAngle
    ) {
        // 1. 목표물까지의 거리 및 각도 계산
        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();
        double distToGoal = Math.hypot(dx, dy); // 수평 거리 x
        double angleToGoal = Math.atan2(dy, dx); // theta_line

        // --- STEP A: 정지 상태(Static) 계산 ---
        // 발사 각도 alpha 계산
        // alpha = atan( (2y/x) - tan(theta_entry) )
        double term1 = (2 * goalHeight) / distToGoal;
        double term2 = Math.tan(targetEntryAngle);
        double alphaStatic = Math.atan(term1 - term2);

        // 초기 속도 v0 계산
        double cosAlpha = Math.cos(alphaStatic);
        double tanAlpha = Math.tan(alphaStatic);
        double numerator = GRAVITY * Math.pow(distToGoal, 2);
        double denominator = 2 * Math.pow(cosAlpha, 2) * ((distToGoal * tanAlpha) - goalHeight);

        if (denominator <= 0) return null; // 도달 불가능
        double v0Static = Math.sqrt(numerator / denominator);

        // 정지 상태 비행 시간 t
        double t = distToGoal / (v0Static * cosAlpha);

        // --- STEP B: 속도 보정(Velocity Compensation) ---

        // 로봇 속도 벡터 크기 및 각도
        double robotVelMag = Math.hypot(robotVel.getXComponent(), robotVel.getYComponent());
        double robotVelAngle = Math.atan2(robotVel.getYComponent(), robotVel.getXComponent());

        // 로봇 속도를 Radial(방사형)과 Tangential(접선)로 분해
        double thetaDiff = robotVelAngle - angleToGoal; //

        // V_rr: 목표물 쪽으로 다가가는 속도
        // (문서에선 -cos이나, 접근 방향 정의에 따라 부호 주의. 여기선 접근을 양수로 가정하거나 문서 공식 따름)
        // 문서: -cos(theta) * mag (다가갈때 뺀다는 의미)
        double vRr = -Math.cos(thetaDiff) * robotVelMag;

        // V_rt: 옆으로 움직이는 속도
        double vRt = Math.sin(thetaDiff) * robotVelMag;

        // X축 속도 보정
        // 원래 가야할 수평 속도 + 로봇이 도와주는(혹은 방해하는) 방사형 속도
        double vX_compensated = (distToGoal / t) + vRr;

        // 새로운 X축 총합 속도 벡터 (V_x,new)
        double vX_new = Math.sqrt(Math.pow(vX_compensated, 2) + Math.pow(vRt, 2));

        // 수직 속도 (V_y)는 로봇 이동과 무관
        double vY = v0Static * Math.sin(alphaStatic);

        // 최종 발사 각도 (alpha_new)
        double alphaNew = Math.atan2(vY, vX_new);

        // 최종 발사 속도 (v_launch_new)
        // 새로운 가상의 거리(x_new) = vX_new * t
        double distNew = vX_new * t;

        double cosAlphaNew = Math.cos(alphaNew);
        double tanAlphaNew = Math.tan(alphaNew);
        double numNew = GRAVITY * Math.pow(distNew, 2);
        double denNew = 2 * Math.pow(cosAlphaNew, 2) * ((distNew * tanAlphaNew) - goalHeight);

        double vLaunchNew = (denNew > 0) ? Math.sqrt(numNew / denNew) : v0Static;

        // 터렛 오프셋 계산 (Turret Offset)
        double turretOffset = Math.atan2(vRt, vX_compensated);

        return new ShotResult(alphaNew, vLaunchNew, turretOffset, t);
    }
}