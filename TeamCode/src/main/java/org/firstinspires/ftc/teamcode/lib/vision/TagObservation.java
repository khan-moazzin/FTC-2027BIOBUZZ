package org.firstinspires.ftc.teamcode.lib.vision;

import org.firstinspires.ftc.teamcode.lib.math.Vec3;

public final class TagObservation {
  public final int id;
  public final Vec3 camera;

  public TagObservation(int id, Vec3 camera) {
    this.id = id;
    this.camera = camera;
  }
}
