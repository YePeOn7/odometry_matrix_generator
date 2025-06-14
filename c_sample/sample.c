#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define ROBOT_RADIUS 1.0
#define WHEEL_RADIUS 0.6
#define HEADING_OFFSET 45

// ---------- Matrix Kinematic ---------- //
float m_kinematic[4][3] = {
    {-1.178511f, 1.178511f, 1.666667f},
    {-1.178511f, -1.178511f, 1.666667f},
    {1.178511f, -1.178511f, 1.666667f},
    {1.178511f, 1.178511f, 1.666667f}
};
// ---------- Matrix Odometry ---------- //
float m_odometry[2][4] = {
    {-0.212132f, -0.212132f, 0.212132f, 0.212132f},
    {0.212132f, -0.212132f, -0.212132f, 0.212132f}
};

void transformKinematic(float x, float y, float w, const float m_kinematic[4][3], float output[4]) {
    for (int i = 0; i < 4; i++) {
        output[i] = m_kinematic[i][0] * x +
                    m_kinematic[i][1] * y +
                    m_kinematic[i][2] * w;
    }
}

void drive(float x, float y, float w, float headingOffset, int numOfWheels, float wheelRadius, float robotRadius, float* motorOutput) {
    float del_angle = 360.0f / numOfWheels;

    for (int i = 0; i < numOfWheels; i++) {
        float angleDeg = del_angle * i + headingOffset;
        float angleRad = angleDeg * M_PI / 180.0f;

        motorOutput[i] = (-x * sinf(angleRad)) / wheelRadius;
        motorOutput[i] += (y * cosf(angleRad)) / wheelRadius;
        motorOutput[i] += (w * robotRadius) / wheelRadius;
    }
}

void odometry(float MI[2][4], float T[4], float angle, float output[2]) {
    // Step 1: Create rotation matrix R (2x2)
    float rad = angle * M_PI / 180.0; // convert degrees to radians
    float R[2][2] = {
        {cos(rad), -sin(rad)},
        {sin(rad),  cos(rad)}
    };

    // Step 3: Multiply MI (2x4) by T (4x1) -> result M (2x1)
    float M[2][1] = {0};

    for(int i = 0; i < 2; i++) {
        M[i][0] = 0;
        for(int j = 0; j < 4; j++) {
            M[i][0] += MI[i][j] * T[j];
        }
    }

    // Step 4: Multiply R (2x2) by M (2x1) -> output (2x1)
    for(int i = 0; i < 2; i++) {
        output[i] = 0;
        for(int j = 0; j < 2; j++) {
            output[i] += R[i][j] * M[j][0];
        }
    }
}

// source: https://www.researchgate.net/publication/348272315_Implementation_Kinematics_Modeling_and_Odometry_of_Four_Omni_Wheel_Mobile_Robot_on_The_Trajectory_Planning_and_Motion_Control_Based_Microcontroller
void odometry2(float T[4], float angle_robot, float output[2]) {
    float gain = 0.6; // ini g ada di papernya, cuma setelah dicoba kayaknya butuh
    float x = 0;
    float y = 0;
    for(int i = 0; i < 4; i++){
        float a_rad = (45 + angle_robot + i * 90)  * M_PI / 180; // 45 karena sudut roda pertama terhadap roda 45, 90 karena jarak masing2 roda 90 
        x += -sinf(a_rad) * T[i] / 2;
        y += cos(a_rad) * T[i] / 2;
    }

    output[0] = x * gain;
    output[1] = y * gain;
}

int main() {
    int numOfWheels = 4;
    float motor[4];
    float x = 12, y = 10, w = 0; //input kecepatan robot
    float headingOffset = 45;
    // float wheelRadius = 0.05f;
    // float robotRadius = 0.2f;

    printf("-------------- Input Speed --------------\n");
    printf("x: %.2f, y: %.2f\n", x, y);

    drive(x, y, w, HEADING_OFFSET, numOfWheels, WHEEL_RADIUS, ROBOT_RADIUS, motor);

    printf("----------- output motor ---------------\n");
    for (int i = 0; i < numOfWheels; i++) {
        printf("Motor[%d] = %.2f\n", i, motor[i]);
    }

    transformKinematic(x, y, w, m_kinematic, motor);
    printf("----------- output motor with matrix transform ---------------\n");
    for (int i = 0; i < numOfWheels; i++) {
        printf("Motor[%d] = %.2f\n", i, motor[i]);
    }

    float speedOutput[2];
    odometry(
      m_odometry, 
      motor,
      0,
      speedOutput
    );
    printf("------------ speed output ------------\n");
    printf("x: %.2f, y: %.2f\n", speedOutput[0], speedOutput[1]);

    odometry2(
      motor,
      0,
      speedOutput
    );
    printf("------------ speed output ------------\n");
    printf("x: %.2f, y: %.2f\n", speedOutput[0], speedOutput[1]);

    return 0;
}