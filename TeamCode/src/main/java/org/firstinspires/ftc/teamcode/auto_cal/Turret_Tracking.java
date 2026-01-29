package org.firstinspires.ftc.teamcode.auto_cal;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SHOOTER_ANGLE_TPR;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.sub_const.shooter_const;

public class Turret_Tracking {
    public Turret_Tracking() {}
    private final double GEAR_RATIO = 105.0/25.0;

    public int fix_to_goal_RED(Pose robot_pos) {
        double dy = RED_GOAL.getY() - robot_pos.getY()/* - shooter_const.turret_offset_y(robot_pos.getHeading())*/;
        double dx = RED_GOAL.getX() - robot_pos.getX()/* - shooter_const.turret_offset_x(robot_pos.getHeading())*/;
        double target_Rad = Math.atan2(dy, dx) - robot_pos.getHeading(); //모두 rad값
        return RadToTicks(target_Rad); //목표 엔코더 tick값 반환
    }


    public int fix_to_goal_BLUE(Pose robot_pos) {
        double dy = BLUE_GOAL.getY() - robot_pos.getY()/* - shooter_const.turret_offset_y(robot_pos.getHeading())*/;
        double dx = BLUE_GOAL.getX() - robot_pos.getX()/* - shooter_const.turret_offset_x(robot_pos.getHeading())*/;
        double target_Rad = Math.atan2(dy, dx) - robot_pos.getHeading(); //모두 rad값 반환 ㅇㅇ
        return RadToTicks(target_Rad);
    }

    public double getTargetHeading(Pose robot_pos) {
        double dy = RED_GOAL.getY() - robot_pos.getY()/* - shooter_const.turret_offset_y(robot_pos.getHeading())*/;
        double dx = RED_GOAL.getX() - robot_pos.getX()/* - shooter_const.turret_offset_x(robot_pos.getHeading())*/;
        return Math.toDegrees(Math.atan2(dy, dx) - robot_pos.getHeading());
    }

    int RadToTicks(double Rad){
        while (Rad > Math.PI) {
            Rad -= 2 * Math.PI;
        }
        while (Rad < -Math.PI) {
            Rad += 2 * Math.PI;
        }

        double ticks = (Rad / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * GEAR_RATIO;

        return (int) ticks;
    }

}
