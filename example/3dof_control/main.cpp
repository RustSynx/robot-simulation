//
// Created by Milo on 2/22/26.
//

#include "mouse.h"
#include "inverse_kinematics.h"
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <thread>

mjModel *m = NULL;
mjData *d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

int main() {
  if (!glfwInit())
    return 1;
  char error[1000] = "Could not load binary model";
  std::string modelPath = std::string(SOURCE_DIR) + "/3dof_control.xml";
  m = mj_loadXML(modelPath.c_str(), 0, error, 1000);
  d = mj_makeData(m);

  mj_resetDataKeyframe(m, d, 0);

  GLFWwindow *window =
      glfwCreateWindow(1200, 900, "3-DOF Arm Simulation", NULL, NULL);
  glfwMakeContextCurrent(window);
  mjv_defaultCamera(&cam);
  cam.distance = 3.0;
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scn, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  bool is_paused = true;
  Mouse mouse(window, m, &cam, &scn);

  while (!glfwWindowShouldClose(window)) {
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
      is_paused = false;
    }

    mj_step(m, d);
    if (!is_paused) {
      double target_x = d->mocap_pos[0];
      double target_y = d->mocap_pos[1];
      double target_z = d->mocap_pos[2];
      InverseKinematics inverse_kinematics;
      JointAngles result = inverse_kinematics.solve_ik_3dof(target_x, target_y, target_z);
      d->ctrl[0] = result.theta1;
      d->ctrl[1] = result.theta2;
      d->ctrl[2] = result.theta3;
      d->ctrl[3] = 0;
      d->ctrl[4] = 0;
      d->ctrl[5] = 0;
      mj_step(m, d);
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  return 0;
}