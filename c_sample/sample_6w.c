#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define ROBOT_RADIUS 1.0
#define WHEEL_RADIUS 0.6
#define HEADING_OFFSET 0
#define NUMBER_OF_WHEELS 6

// ---------- Matrix Kinematic ---------- //
float m_kinematic[6][3] = {
    {-0.000000f, 1.666667f, 1.666667f},
    {-1.443376f, 0.833333f, 1.666667f},
    {-1.443376f, -0.833333f, 1.666667f},
    {-0.000000f, -1.666667f, 1.666667f},
    {1.443376f, -0.833333f, 1.666667f},
    {1.443376f, 0.833333f, 1.666667f}
};
// ---------- Matrix Odometry ---------- //
float m_odometry[2][6] = {
    {0.000000f, -0.173205f, -0.173205f, -0.000000f, 0.173205f, 0.173205f},
    {0.200000f, 0.100000f, -0.100000f, -0.200000f, -0.100000f, 0.100000f}
};
void transformKinematic(float x, float y, float w, const float m_kinematic[4][3], float output[4]) {
  for (int i = 0; i < 4; i++) {
    output[i] = m_kinematic[i][0] * x +
                m_kinematic[i][1] * y +
                m_kinematic[i][2] * w;
  }
}

void drive(float x, float y, float w, float headingOffset, int n, float wheelRadius, float robotRadius, float* motorOutput) {
  float del_angle = 360.0f / n;

  for (int i = 0; i < n; i++) {
    float angleDeg = del_angle * i + headingOffset;
    float angleRad = angleDeg * M_PI / 180.0f;

    motorOutput[i] = (-x * sinf(angleRad)) / wheelRadius;
    motorOutput[i] += (y * cosf(angleRad)) / wheelRadius;
    motorOutput[i] += (w * robotRadius) / wheelRadius;
  }
}

void odometry(int N, float* MI, float* T, float angle, float* output) {
  float rad = angle * M_PI / 180.0f;

  // Rotation matrix
  float R[2][2] = {
      {cosf(rad), -sinf(rad)},
      {sinf(rad), cosf(rad)}};

  // Matrix multiply: M = MI * T (2xN * Nx1 = 2x1)
  float M[2] = {0};

  for (int i = 0; i < 2; i++) {
    M[i] = 0;
    for (int j = 0; j < N; j++) {
      M[i] += MI[i * N + j] * T[j];
    }
  }

  // Multiply R * M -> output (2x1)
  for (int i = 0; i < 2; i++) {
    output[i] = 0;
    for (int j = 0; j < 2; j++) {
      output[i] += R[i][j] * M[j];
    }
  }
}

// source: https://www.researchgate.net/publication/348272315_Implementation_Kinematics_Modeling_and_Odometry_of_Four_Omni_Wheel_Mobile_Robot_on_The_Trajectory_Planning_and_Motion_Control_Based_Microcontroller,
// tapi dibuat lebih general
void odometry2(int N, float* T, float angle_robot, float output[2], float heading_offset) {
  float gain = 0.4;  // ini g ada di papernya, cuma setelah dicoba kayaknya butuh
  float x = 0;
  float y = 0;
  float d_angle = 360 / (float)N;

  for (int i = 0; i < N; i++) {
    float a_rad = (heading_offset + angle_robot + i * d_angle) * M_PI / 180;
    x += -sinf(a_rad) * T[i] / 2;
    y += cosf(a_rad) * T[i] / 2;
  }

  output[0] = x * gain;
  output[1] = y * gain;
}

int main() {
  float motor[4];
  float x = 12, y = 20, w = 0;  // input kecepatan robot
  float headingOffset = 45;
  // float wheelRadius = 0.05f;
  // float robotRadius = 0.2f;

  printf("-------------- Input Speed --------------\n");
  printf("x: %.2f, y: %.2f\n", x, y);

  drive(x, y, w, HEADING_OFFSET, NUMBER_OF_WHEELS, WHEEL_RADIUS, ROBOT_RADIUS, motor);

  printf("----------- output motor ---------------\n");
  for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
    printf("Motor[%d] = %.2f\n", i, motor[i]);
  }

  transformKinematic(x, y, w, m_kinematic, motor);
  printf("----------- output motor with matrix transform ---------------\n");
  for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
    printf("Motor[%d] = %.2f\n", i, motor[i]);
  }

  float speedOutput[2];
  odometry(
      NUMBER_OF_WHEELS,
      (float*)m_odometry,
      motor,
      0,
      speedOutput);
  printf("------------ speed output odometry 1------------\n");
  printf("x: %.2f, y: %.2f\n", speedOutput[0], speedOutput[1]);

  odometry2(
      NUMBER_OF_WHEELS,
      motor,
      HEADING_OFFSET,
      speedOutput,
      0);
  printf("------------ speed output odometry 2 (from papaer but add gain)------------\n");
  printf("x: %.2f, y: %.2f\n", speedOutput[0], speedOutput[1]);

  return 0;
}