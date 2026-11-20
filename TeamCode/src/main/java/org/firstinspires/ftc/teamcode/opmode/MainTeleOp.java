package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Wedge;

/**
 * 25323 대회용 TeleOp.
 *
 * <pre>
 * gamepad1 (드라이버)
 *   왼쪽 스틱      이동 (필드 기준: 스틱 위 = 시작할 때 로봇이 보던 쪽)
 *   오른쪽 스틱 X  회전
 *   RT 누르고 있기 느린 모드
 *   OPTIONS        지금 방향을 "앞"으로 다시 정하기 (IMU 리셋)
 *   SHARE          필드 기준 ↔ 로봇 기준 주행 바꾸기 (IMU가 이상할 때 비상용)
 *
 * gamepad2 (오퍼레이터)
 *   RT 누르고 있기 발사: 블로커 열림 + 슈터 가속 → 속도 도달하면 인테이크가 공을 밀어 넣음 → 놓으면 정지
 *   A / B          인테이크 줍기 / 뱉기 (누르는 동안)
 *   D-pad 좌 / 우  블로커 열기 / 닫기
 *   D-pad 상 / 하  쐐기 내밀기 / 접기
 *   오른쪽 스틱 X  터렛 돌리기
 *   LB / RB        터렛 천천히 왼쪽 / 오른쪽 (누르는 동안)
 *   Y              슈터 예열 켜기/끄기 (기본 켜짐)
 * </pre>
 */
@TeleOp(name = "25323 TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Drive drive = new Drive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Wedge wedge = new Wedge(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        telemetry.addLine("준비 완료. 로봇을 경기장 안쪽(앞)으로 향하게 놓고 START.");
        telemetry.update();
        waitForStart();

        drive.resetHeading();
        intake.closeBlocker();
        boolean preheat = true;
        boolean wasShooting = false;
        boolean feeding = false;

        while (opModeIsActive()) {
            // ── 주행 (gamepad1) ──
            if (gamepad1.optionsWasPressed()) drive.resetHeading();
            if (gamepad1.shareWasPressed()) drive.setFieldCentric(!drive.isFieldCentric());
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
                    gamepad1.right_trigger > 0.5);

            // ── 발사 (gamepad2 RT) ──
            if (gamepad2.yWasPressed()) preheat = !preheat;
            boolean shooting = gamepad2.right_trigger > 0.5;
            if (shooting) {
                if (!wasShooting) {
                    intake.openBlocker();
                    feeding = false;
                }
                shooter.setMode(Shooter.Mode.SHOOT);
                if (!feeding && shooter.atShootSpeed()) feeding = true;   // 한 번 도달하면 놓을 때까지 계속 급탄
            } else {
                if (wasShooting) intake.closeBlocker();
                feeding = false;
                shooter.setMode(preheat ? Shooter.Mode.IDLE : Shooter.Mode.OFF);
            }
            wasShooting = shooting;
            shooter.update();

            // ── 인테이크 (A/B가 발사 급탄보다 우선) ──
            if (gamepad2.a) intake.intake();
            else if (gamepad2.b) intake.outtake();
            else if (feeding) intake.feed();
            else intake.stop();

            // ── 블로커·쐐기 수동 ──
            if (gamepad2.dpadLeftWasPressed()) intake.openBlocker();
            if (gamepad2.dpadRightWasPressed()) intake.closeBlocker();
            if (gamepad2.dpadUpWasPressed()) wedge.extend();
            if (gamepad2.dpadDownWasPressed()) wedge.retract();

            // ── 터렛 ──
            if (gamepad2.left_bumper) turret.fine(-1);
            else if (gamepad2.right_bumper) turret.fine(1);
            else turret.manual(Math.abs(gamepad2.right_stick_x) < 0.05 ? 0 : gamepad2.right_stick_x);

            // ── 화면 ──
            telemetry.addData("주행", "%s  방향 %.1f°", drive.isFieldCentric() ? "필드 기준" : "로봇 기준", drive.headingDeg());
            telemetry.addData("슈터", "%.0f / %.0f  %s  (예열 %s)", shooter.velocity(), shooter.targetVelocity(),
                    shooting ? (feeding ? "급탄 중" : "가속 중") : "-", preheat ? "켜짐" : "꺼짐");
            telemetry.addData("블로커", intake.blockerOpen() ? "열림" : "닫힘");
            telemetry.addData("쐐기", wedge.isExtended() ? "내밂" : "접음");
            telemetry.addData("터렛 파워", "%.2f", turret.power());
            telemetry.update();
        }

        drive.stop();
        shooter.stop();
        intake.stop();
        turret.stop();
    }
}
