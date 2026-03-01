//
// Created by Milo on 3/1/26.
//

#ifndef INC_3DOF_CONTROL_MOUSE_H
#define INC_3DOF_CONTROL_MOUSE_H
#include "GLFW/glfw3.h"
#include <mujoco/mujoco.h>

class Mouse {
private:
  double lastx = 0;
  double lasty = 0;
  bool button_left = false;
  bool button_right = false;
  bool button_middle = false;
  mjModel *m;
  mjvCamera *cam;
  mjvScene *scn;
public:
  Mouse(GLFWwindow *window, mjModel *m, mjvCamera *cam, mjvScene *scn);
  static void mouse_button_callback(GLFWwindow *window, int button, int action,
                                    int mods);

  void mouse_button(GLFWwindow *window, int button, int action, int mods);

  static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);

  void mouse_move(GLFWwindow *window, double xpos, double ypos);

  static void mouse_scroll_callback(GLFWwindow *window, double xoffset,
                                    double yoffset);

  void scroll(GLFWwindow *window, double xoffset, double yoffset);
};

#endif // INC_3DOF_CONTROL_MOUSE_H
