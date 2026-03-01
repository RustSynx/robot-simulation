//
// Created by Milo on 3/1/26.
//

#include "mouse.h"
#include "inverse_kinematics.h"
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <thread>
#include <iostream>

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
  std::string modelPath = std::string(SOURCE_DIR) + "/pick_and_place.xml";
  m = mj_loadXML(modelPath.c_str(), 0, error, 1000);
  d = mj_makeData(m);

  mj_resetDataKeyframe(m, d, 0);

  GLFWwindow *window =
      glfwCreateWindow(1200, 900, "Pick and Place", NULL, NULL);
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

  int target_id = mj_name2id(m, mjOBJ_BODY, "target");
  int target_spot_id = mj_name2id(m, mjOBJ_SITE, "target_spot");
  std::string state = "WAIT";
  InverseKinematics inverse_kinematics;
  JointAngles result{};
  while (!glfwWindowShouldClose(window)) {
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
      is_paused = false;
    }

    if (!is_paused) {
      double target_x = d->xpos[3 * target_id + 0];
      double target_y = d->xpos[3 * target_id + 1];
      double target_z = d->xpos[3 * target_id + 2];
      if (state == "WAIT") {
        result = inverse_kinematics.solve_ik_3dof(target_x, target_y, target_z);
        d->ctrl[0] = result.theta1;
        d->ctrl[1] = result.theta2;
        d->ctrl[2] = result.theta3;
        d->ctrl[3] = -2.3462;
        d->ctrl[4] = 0;
        d->ctrl[5] = 0;
      }
      double epsilon = 1e-3;
      if (state == "WAIT" && abs(d->qpos[7] - result.theta1) < epsilon) {
        d->ctrl[6] = 300;
        state = "PICK";
      }
      if (state == "MOVE" && (abs(d->qpos[7] - result.theta1) < epsilon)) {
        std::cout << "MOVE!!" << std::endl;
        d->ctrl[6] = 0;
        state = "END";
      }
      if (state == "PICK" && d->qpos[13] > 0.476) {
        double target_spot_x = d->site_xpos[3 * target_spot_id + 0];
        double target_spot_y = d->site_xpos[3 * target_spot_id + 1];
        double target_spot_z = d->site_xpos[3 * target_spot_id + 2];
        result = inverse_kinematics.solve_ik_3dof(target_spot_x, target_spot_y, target_spot_z);
        d->ctrl[0] = result.theta1;
        d->ctrl[1] = result.theta2;
        d->ctrl[2] = result.theta3;
        d->ctrl[3] = -2.3462;
        d->ctrl[4] = 0;
        d->ctrl[5] = 0;
        d->ctrl[6] = 300;
        state = "MOVE";
      }
    }
    mj_step(m, d);

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