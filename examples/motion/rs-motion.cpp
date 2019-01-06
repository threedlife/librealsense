#include <librealsense2/rs.hpp>
//#include <algorithm>
#include <iostream>
#include <thread>
//#include <../third-party/eigen/Eigen/Geometry>

#include <mutex>

#include "example.hpp"          // Include short list of convenience functions for rendering
#include "rendering.h"
#include "rs_export.hpp"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

struct int3
{
    int x, y, z;
};

#include "d435.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


class GyroBias
{
private:
    int calibrationUpdates;
    double minX, maxX;
    double minY, maxY;
    double minZ, maxZ;


public:
    bool isSet;

    double x;
    double y;
    double z;

    GyroBias()
    {
        reset();
    }

    void reset()
    {
        calibrationUpdates = 0;

        minX = 1000;
        minY = 1000;
        minZ = 1000;
        maxX = -1000;
        maxY = -1000;
        maxZ = -1000;

        x = 0;
        y = 0;
        z = 0;

        isSet = false;
    }


    bool update(double gx, double gy, double gz)
    {
        if (calibrationUpdates < 50)
        {
            maxX = std::max(gx, maxX);
            maxY = std::max(gy, maxY);
            maxZ = std::max(gz, maxZ);

            minX = std::min(gx, minX);
            minY = std::min(gy, minY);
            minZ = std::min(gz, minZ);

            calibrationUpdates++;
            return false;
        }
        else if (calibrationUpdates == 50)
        {
            x = (maxX + minX) / 2.0;
            y = (maxY + minY) / 2.0;
            z = (maxZ + minZ) / 2.0;
            calibrationUpdates++;
            isSet = true;
           /* std::cout << "BIAS-X: " << minX << " - " << maxX << std::endl;
            std::cout << "BIAS-Y: " << minY << " - " << maxY << std::endl;
            std::cout << "BIAS-Z: " << minZ << " - " << maxZ << std::endl;
            std::cout << "BIAS: " << x << " " << y << " " << z << std::endl; */
            return true;
        }
        else
        {
            return false;
        }
    }
};

int main()
{

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);

    //cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    float thetaX = 0.0;
    float thetaY = 0.0;
    float thetaZ = 0.0;

    GyroBias bias;

    // Start capturing
    auto profile = pipe.start(cfg);

    bool firstAccel = true;
    double last_ts[RS2_STREAM_COUNT];
    double dt[RS2_STREAM_COUNT];
   // Eigen::Affine3f rot;
    rs2::matrix4 rot;
  //  std::mutex mtx;
    bool rot_valid = false;

    window app(1280, 720, "RealSense Motion Example");
    // Construct an object to manage view state
    glfw_state app_state(0.0, 0.0);
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);


    std::vector<float3> positions;
    std::vector<float3> normals;
    std::vector<int3> indexes;
    uncompress_d435_obj(positions, normals, indexes);

    double sx = 0.;
    for (auto& xyz : positions)
    {
        xyz.x = (xyz.x - 0 * 1000.f) / 30.f;
        xyz.y = (xyz.y - 0 * 1000.f) / 30.f;
        xyz.z = (xyz.z - 0 * 1000.f) / 30.f;
        sx += xyz.x;
    }
    sx /= positions.size();
  //  float prev = 0;
  //  float accelX = 0;

    rs2::rates_printer p;
    float alpha = 0.98;
    rs2::estimate_motion em;

    // Main loop
    while (app)
    {
        glfwSwapInterval(0);

        rs2::frameset frames;
        rs2_quaternion rot_q;

        frames = pipe.wait_for_frames();


        auto fa = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
       // rs2::motion_frame accel = fa.as<rs2::motion_frame>();

        auto fg = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        //rs2::motion_frame gyro = fg.as<rs2::motion_frame>();

        rs2::pose_frame pose = em.process(fa); // TODO: same with fg?
        auto pose_data = pose.get_pose_data();
        auto rot_q = pose_data.rotation;
        rs2::matrix4 rot(rot_q);

        // Set the current clear color to black and the current drawing color to
        // white.
        glClearColor(0.0, 0.0, 0.0, 1.0);
        glColor3f(1.0, 1.0, 1.0);

        // Set the camera lens to have a 60 degree (vertical) field of view, an
        // aspect ratio of 4/3, and have everything closer than 1 unit to the
        // camera and greater than 40 units distant clipped away.
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, 4.0 / 3.0, 1, 40);

        glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_MODELVIEW);

        glLoadIdentity();
        gluLookAt(sx, 0, 5, sx, 0, 0, 0, 1, 0);
        //gluLookAt(sx, 0, -5, sx, 0, 0, 0, 1, 0);

        glTranslatef(0, 0, +0.5f + app_state.offset_y*0.15f);
        glRotated(app_state.pitch, 1, 0, 0);
        glRotated(app_state.yaw, 0, -1, 0);

        glColor3f(1.0, 1.0, 1.0);

        glLineWidth(1);
        glBegin(GL_LINES);
        glColor4f(0.4f, 0.4f, 0.4f, 1.f);

        // Render "floor" grid
        for (int i = 0; i <= 8; i++)
        {
            glVertex3i(i - 4, -1, 0);
            glVertex3i(i - 4, -1, 8);
            glVertex3i(-4, -1, i);
            glVertex3i(4, -1, i);
        }
        glEnd();

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(rot.mat[0][0], rot.mat[0][1], rot.mat[0][2]);
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(rot.mat[1][0], rot.mat[1][1], rot.mat[1][2]);
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(rot.mat[2][0], rot.mat[2][1], rot.mat[2][2]);
        glEnd();

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);

        float curr[4][4];
        float mult[4][4];
        float mat[4][4] = {
            { rot.mat[0][0], rot.mat[0][1], rot.mat[0][2], rot.mat[0][3] },
            { rot.mat[1][0], rot.mat[1][1], rot.mat[1][2], rot.mat[1][3] },
            { rot.mat[2][0], rot.mat[2][1], rot.mat[2][2], rot.mat[2][3] },
            { rot.mat[3][0], rot.mat[3][1], rot.mat[3][2], rot.mat[3][3] }
        };

        glGetFloatv(GL_MODELVIEW_MATRIX, (float*)curr);

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                auto sum = 0.f;
                for (int k = 0; k < 4; k++)
                {
                    sum += mat[i][k] * curr[k][j];
                }
                mult[i][j] = sum;
            }

        glLoadMatrixf((float*)mult);

        glBegin(GL_TRIANGLES);
        for (auto& i : indexes)
        {
            auto v0 = positions[i.x];
            auto v1 = positions[i.y];
            auto v2 = positions[i.z];
            glVertex3fv(&v0.x);
            glVertex3fv(&v1.x);
            glVertex3fv(&v2.x);
            glColor4f(0.05f, 0.05f, 0.05f, 0.3f);
        }
        glEnd();

        glDisable(GL_BLEND);

        glFlush();
    }

    return 0;
}
