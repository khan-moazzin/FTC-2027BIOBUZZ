package org.firstinspires.ftc.teamcode.lib.calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

/** Edge-triggered operator acknowledgements; each system has its own OpMode. */
public abstract class GuidedOpMode extends OpMode {
  protected boolean a, b, x, y;
  private boolean pa, pb, px, py;
  protected String result = "";

  @Override
  public final void init() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    try {
      setup();
    } catch (RuntimeException failure) {
      try {
        stop();
      } catch (RuntimeException cleanup) {
        failure.addSuppressed(cleanup);
      }
      throw failure;
    }
  }

  protected abstract void setup();

  @Override
  public final void loop() {
    a = gamepad1.a && !pa;
    b = gamepad1.b && !pb;
    x = gamepad1.x && !px;
    y = gamepad1.y && !py;
    try {
      tick(System.nanoTime());
    } catch (Exception e) {
      result = e.getClass().getSimpleName() + ": " + e.getMessage();
      try {
        shutdown();
      } finally {
        requestOpModeStop();
      }
    }
    pa = gamepad1.a;
    pb = gamepad1.b;
    px = gamepad1.x;
    py = gamepad1.y;
    telemetry.addData("Result", result);
    telemetry.update();
  }

  protected abstract void tick(long now) throws Exception;

  protected abstract void shutdown();

  protected void export(Object config) throws Exception {
    result = "Copy " + ConfigExport.save(config).getAbsolutePath();
  }

  @Override
  public final void stop() {
    try {
      shutdown();
    } finally {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().reset();
    }
  }
}
