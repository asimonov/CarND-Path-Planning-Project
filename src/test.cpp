//
// Created by Alexey Simonov on 30/09/2017.
//

public void FixedUpdate() {
  if (main_car) {
    //average over last frames
    int time_steps = 10;
    float speed = m_Rigidbody.velocity.magnitude;
    if (averageSpeed.Count >= time_steps) {

      float averaged_speed = AverageLastSpeed();
      AccelerationT = (averaged_speed - lastSpeed) / (time_steps * Time.deltaTime);
      AccelerationN = (averaged_speed) * (averaged_speed) * SenseCurve();
      lastSpeed = averaged_speed;
      float currentAcc = SenseAcc();
      averageAcc.Add(currentAcc);
      averageSpeed.Clear();
      previous_pos.Clear();
    }
    averageSpeed.Add(speed);
    previous_pos.Add(transform.position);
    if (averageAcc.Count >= 5) {
      float averaged_acc = AverageLastAcc();
      Jerk = (averaged_acc - lastAcc) / ((5 * time_steps) * Time.deltaTime);
      lastAcc = averaged_acc;
      averageAcc.Clear();
    }

  }
}


public float AverageLastSpeed() {

  float averaged_speed = 0.0f;
  for (int i = 0; i < averageSpeed.Count; i++) {
    averaged_speed += averageSpeed[i];
  }
  return averaged_speed / (float) (averageSpeed.Count);
}


public float AverageLastAcc() {
  float averaged_acc = 0.0f;
  for (int i = 0; i < averageAcc.Count; i++) {
    averaged_acc += averageAcc[i];
  }
  return averaged_acc / (float) (averageAcc.Count);
}


public float SenseCurve() {
  float averaged_curve = 0.0f;
  for (int i = 0; i < previous_pos.Count - 2; i++) {

    float x1 = previous_pos[i].x;
    float x2 = previous_pos[i + 1].x;
    float x3 = previous_pos[i + 2].x;
    float y1 = previous_pos[i].z;
    float y2 = previous_pos[i + 1].z;
    float y3 = previous_pos[i + 2].z;
    Vector2 ray1 = new Vector2(x2 - x1, y2 - y1);
    Vector2 ray2 = new Vector2(x3 - x2, y3 - y2);
    if (ray1.magnitude != 0 && ray2.magnitude != 0) {
      Vector2 ray3 = new Vector2(x3 - x1, y3 - y1);
      float corner_angle = Mathf.Abs(Vector2.Angle(ray1, ray2));
      if (ray3.magnitude != 0 && corner_angle != 180) {
        averaged_curve += 2 * Mathf.Sin(corner_angle * Mathf.Deg2Rad) / ray3.magnitude;
      } else {

        //the curve is infinite, this move is totally illegal, just going to return 1000000
        averaged_curve += 1000000;
      }
    }
    //else skip and just say that curve is zero since its stopped
  }
  return averaged_curve / ((float) (previous_pos.Count - 2));
}