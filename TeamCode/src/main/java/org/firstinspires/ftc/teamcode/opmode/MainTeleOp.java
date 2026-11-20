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
 *   RT 누르고 있기 발사: 블로커 열림 + 쐐기 발사 위치 + 슈터 가속 → 속도 도달하면 인테이크가 공을 밀어 넣음
 *                  → 놓으면 정지, 쐐기는 발사 전 상태로 돌아감
 *   A / B          인테이크 줍기 / 뱉기 (누르는 동안)
 *   D-pad 좌 / 우  블로커 열기 / 닫기
 *   D-pad 상 / 하  쐐기 한 단계 내밀기 / 접기 (높게 접음 ↔ 낮게 접음(발사) ↔ 내밂(빼오기))
 *   오른쪽 스틱 X  터렛 돌리기
 *   LB / RB        터렛 천천히 왼쪽 / 오른쪽 (누르는 동안)
 *   X              터렛 가운데(인테이크 방향)로 복귀
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
        if (Turret.POSITION_MODE) telemetry.addLine("START 하면 터렛이 가운데로, 쐐기가 높게 접은 위치로 움직임. 주변 확인.");
        else if (turret.hasFeedback()) telemetry.addLine("터렛을 가운데(인테이크 방향)에 맞춘 상태에서 INIT 했는지 확인.");
        telemetry.update();
        waitForStart();

        drive.resetHeading();
        intake.closeBlocker();
        wedge.set(Wedge.State.STOW);
        Wedge.State wedgeBeforeShot = null;
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
                    wedgeBeforeShot = wedge.state();
                    wedge.set(Wedge.State.SHOOT);
                    feeding = false;
                }
                shooter.setMode(Shooter.Mode.SHOOT);
                if (!feeding && shooter.atShootSpeed()) feeding = true;   // 한 번 도달하면 놓을 때까지 계속 급탄
            } else {
                if (wasShooting) {
                    intake.closeBlocker();
                    if (wedgeBeforeShot != null) wedge.set(wedgeBeforeShot);
                }
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
            if (gamepad2.dpadUpWasPressed()) wedge.stepOut();
            if (gamepad2.dpadDownWasPressed()) wedge.stepIn();

            // ── 터렛 ──
            if (gamepad2.xWasPressed()) turret.center();
            int fine = gamepad2.left_bumper ? -1 : gamepad2.right_bumper ? 1 : 0;
            turret.update(Math.abs(gamepad2.right_stick_x) < 0.05 ? 0 : gamepad2.right_stick_x, fine);

            // ── 화면 ──
            telemetry.addData("주행", "%s  방향 %.1f°", drive.isFieldCentric() ? "필드 기준" : "로봇 기준", drive.headingDeg());
            telemetry.addData("슈터", "%.0f / %.0f  %s  (예열 %s)", shooter.velocity(), shooter.targetVelocity(),
                    shooting ? (feeding ? "급탄 중" : "가속 중") : "-", preheat ? "켜짐" : "꺼짐");
            telemetry.addData("블로커", intake.blockerOpen() ? "열림" : "닫힘");
            telemetry.addData("쐐기", Wedge.label(wedge.state()));
            if (Turret.POSITION_MODE) {
                telemetry.addData("터렛", "위치 %.3f  (가운데 %.3f, 범위 %.3f~%.3f) %s", turret.targetPosition(),
                        Turret.CENTER_POS, Turret.MIN_POS, Turret.MAX_POS, turret.atLimit() ? "■ 끝" : "");
            } else if (turret.angleKnown()) {
                telemetry.addData("터렛", "%.1f°  파워 %.2f  %s%s", turret.angleDeg(), turret.power(),
                        turret.isCentering() ? "가운데로 복귀 중 " : "", turret.atLimit() ? "■ 한계각" : "");
            } else {
                telemetry.addData("터렛", "파워 %.2f  (%s)", turret.power(),
                        turret.hasFeedback() ? "기어비 미입력 → 각도 모름" : "피드백 없음 → 수동만");
            }
            telemetry.update();
        }

        drive.stop();
        shooter.stop();
        intake.stop();
        turret.stop();
    }
}
