#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "Serial.h"
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <clocale>

struct SensorData
{
    float w, x, y, z;
};

bool parseData(const std::string &data, SensorData &s1, SensorData &s2, SensorData &s3)
{
    float v[12];

    int count = sscanf(data.c_str(), "%f;%f;%f;%f|%f;%f;%f;%f|%f;%f;%f;%f",
                       &v[0], &v[1], &v[2], &v[3],
                       &v[4], &v[5], &v[6], &v[7],
                       &v[8], &v[9], &v[10], &v[11]);

    if (count != 12)
        return false;

    s1 = {v[0], v[1], v[2], v[3]};
    s2 = {v[4], v[5], v[6], v[7]};
    s3 = {v[8], v[9], v[10], v[11]};
    return true;
}

int main()
{
    setlocale(LC_NUMERIC, "C");

    Serial sp = Serial("COM3"); 

    const float ARM_LENGTH = 4.0f;
    const float ARM_WIDTH = 0.8f;

    InitWindow(1200, 800, "Wizualizacja 3D");
    SetTargetFPS(60);

    float cameraAngle = 0.0f;
    Camera3D camera = {0};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 80.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Tworzenie modelu klocka
    Mesh mesh = GenMeshCube(ARM_WIDTH, ARM_LENGTH, ARM_WIDTH);
    Model model = LoadModelFromMesh(mesh);

    if (sp.isConnected())
    {
        std::cout << "Reset Arduino..." << std::endl;
        Sleep(1000);
    }

    char buffer[1024] = "";
    std::string lineBuffer = "";
    SensorData mpu1 = {1, 0, 0, 0};
    SensorData mpu2 = {1, 0, 0, 0};
    SensorData mpu3 = {1, 0, 0, 0};
    while (!WindowShouldClose())
    {
        // Kamera
        if (IsKeyDown(KEY_LEFT))
            cameraAngle -= 0.03f;
        if (IsKeyDown(KEY_RIGHT))
            cameraAngle += 0.03f;
        camera.position.x = sin(cameraAngle) * 15.0f;
        camera.position.z = cos(cameraAngle) * 15.0f;
        camera.position.y = 10.0f;
        camera.target = (Vector3){0.0f, 4.0f, 0.0f};

        // Odczyt
        if (sp.isConnected())
        {
            int bytesRead = sp.readData(buffer, 1023);
            if (bytesRead > 0)
            {
                buffer[bytesRead] = 0;
                lineBuffer += buffer;
                size_t pos = lineBuffer.find('\n');
                while (pos != std::string::npos)
                {
                    std::string line = lineBuffer.substr(0, pos);
                    lineBuffer.erase(0, pos + 1);
                    parseData(line, mpu1, mpu2, mpu3);
                    pos = lineBuffer.find('\n');
                }
            }
        }

        Quaternion q1_raw = {mpu1.w, mpu1.x, mpu1.z, mpu1.y};
        Quaternion q2_raw = {mpu2.w, mpu2.x, mpu2.z, mpu2.y};
        Quaternion q3_raw = {mpu3.w, mpu3.x, mpu3.z, mpu3.y};
        // --- 1. PRZYGOTOWANIE ---

        Vector3 euler1 = QuaternionToEuler(q1_raw);
        Vector3 euler2 = QuaternionToEuler(q2_raw);
        Vector3 euler3 = QuaternionToEuler(q3_raw);
        Matrix matOffset1 = MatrixTranslate(0.0f, 2.0f, 0.0f);
        Matrix matOffset2 = MatrixTranslate(0.0f, 0.0f, 0.0f);

// człon nr. 1
Matrix matCube1 = MatrixMultiply(MatrixTranslate(0.0f, -2.0f, 0.0f), MatrixRotateXYZ((Vector3){euler1.x, 0, euler1.z}));
// człon nr. 2
Vector3 jointPosition = Vector3Transform((Vector3){0.0f, -4.0f, 0.0f}, MatrixRotateXYZ((Vector3){euler1.x, 0, euler1.z}));
Matrix matCube2temp = MatrixMultiply(MatrixTranslate(0.0f, -2.0f, 0.0f), MatrixRotateXYZ((Vector3){euler2.x, 0, euler2.z}));
Matrix matCube2 = MatrixMultiply(matCube2temp, MatrixTranslate(jointPosition.x, jointPosition.y, jointPosition.z));
// człon nr. 3
Vector3 limb2Vector = Vector3Transform((Vector3){0.0f, -4.0f, 0.0f}, MatrixRotateXYZ((Vector3){euler2.x, 0, euler2.z}));
Matrix matCube3Temp = MatrixMultiply(MatrixTranslate(0.0f, -2.0f, 0.0f), MatrixRotateXYZ((Vector3){euler3.x, 0, euler3.z}));
Vector3 jointPosition2 = Vector3Add(jointPosition, limb2Vector);
Matrix matCube3 = MatrixMultiply(matCube3Temp, MatrixTranslate(jointPosition2.x, jointPosition2.y, jointPosition2.z));

        // --- RYSOWANIE ---
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
        DrawGrid(20, 1.0f);

        model.transform = matCube1;
        DrawModel(model, (Vector3){0, 0, 0}, 1.0f, RED);
        model.transform = matCube2;
        DrawModel(model, (Vector3){0, 0, 0}, 1.0f, BLUE);
        model.transform = matCube3;
        DrawModel(model, (Vector3){0, 0, 0}, 1.0f, GREEN);

        EndMode3D();

        char debug[100];
        // Wyświetlamy kąty w stopniach dla ułatwienia
        sprintf(debug, "MPU: 1 Pitch: %.1f, Yaw: %.1f, Roll: %.1f ", 1, euler1.z * RAD2DEG, euler1.x * RAD2DEG, euler1.y * RAD2DEG);
        DrawText(debug, 10, 30, 20, DARKGRAY);
        sprintf(debug, "MPU: 2 Pitch: %.1f, Yaw: %.1f, Roll: %.1f ", 2, euler2.z * RAD2DEG, euler2.x * RAD2DEG, euler2.y * RAD2DEG);
        DrawText(debug, 10, 60, 20, DARKGRAY);
        sprintf(debug, "MPU: 3 Pitch: %.1f, Yaw: %.1f, Roll: %.1f ", 2, euler3.z * RAD2DEG, euler3.x * RAD2DEG, euler3.y * RAD2DEG);
        DrawText(debug, 10, 90, 20, DARKGRAY);
        EndDrawing();
    }

    UnloadModel(model);
    sp.closeSerial();
    CloseWindow();
    return 0;
}