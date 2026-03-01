//
// Created by Milo on 3/1/26.
//

#include "mouse.h"

Mouse::Mouse(GLFWwindow *window, mjModel *m, mjvCamera *cam, mjvScene *scn) {
  this->m = m;
  this->cam = cam;
  this->scn = scn;
  glfwSetWindowUserPointer(window, this);

  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, mouse_move_callback);
  glfwSetScrollCallback(window, mouse_scroll_callback);
}

void Mouse::mouse_button_callback(GLFWwindow *window, int button, int action,
                                  int mods) {
  Mouse *mouse = static_cast<Mouse*>(glfwGetWindowUserPointer(window));

  if (mouse) {
    mouse->mouse_button(window, button, action, mods);
  }
}

void Mouse::mouse_button(GLFWwindow *window, int button, int action, int mods) {
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);

  glfwGetCursorPos(window, &lastx, &lasty);
}

void Mouse::mouse_move_callback(GLFWwindow *window, double xpos, double ypos) {
  Mouse *mouse = static_cast<Mouse*>(glfwGetWindowUserPointer(window));

  if (mouse) {
    mouse->mouse_move(window, xpos, ypos);
  }
}

void Mouse::mouse_move(GLFWwindow *window, double xpos, double ypos) {
  if (!button_left && !button_right && !button_middle)
    return;

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  mjtMouse action;
  if (button_right)
    action = mjMOUSE_MOVE_V;
  else if (button_left)
    action = mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(m, action, dx / height, dy / height, scn, cam);
}

void Mouse::mouse_scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  Mouse *mouse = static_cast<Mouse*>(glfwGetWindowUserPointer(window));

  if (mouse) {
    mouse->scroll(window, xoffset, yoffset);
  }
}


void Mouse::scroll(GLFWwindow *window, double xoffset, double yoffset) {
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, scn, cam);
}
