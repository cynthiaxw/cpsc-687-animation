#pragma once
#include "io.h"

int model_n = 0;    // behaviours 0-normal, 1-obstacle, 2-bait ball
int N = 4;
int reset = 0;

template <typename CameraT>
class TurnTableControls {
    public:
        TurnTableControls(
            io::Window &w,
            CameraT &c
        ) : m_window(w)
          , m_camera(c)
        {
          m_window.keyboardCommands()
              | io::Key(GLFW_KEY_ESCAPE,
                  [&](auto const &/*event*/) { m_window.shouldClose(); })
              | io::Key(GLFW_KEY_X,
                  [&](auto const &event) {
                    if (event.action == GLFW_PRESS)
                        xLock = !xLock;
                  })
              | io::Key(GLFW_KEY_Y,
                  [&](auto const &event) {
                    if (event.action == GLFW_PRESS)
                        yLock = !yLock;
                  })
              | io::Key(GLFW_KEY_TAB,
                  [&](auto const& event) {
                      if (event.action == GLFW_PRESS)
                          model_n = (model_n + 1) % N;
                  })
              | io::Key(GLFW_KEY_R,
                  [&](auto const& event) {
                      if (event.action == GLFW_PRESS)
                          reset = !reset ? 1 : reset;
                  });

          m_window.mouseCommands() | //
              io::MouseButton(     //
                  GLFW_MOUSE_BUTTON_LEFT, [&](auto const &event) {
                    if (event.action == GLFW_PRESS)
                        cursorLocked = true;
                    else
                        cursorLocked = false;

                    if (event.mods == GLFW_MOD_SHIFT)
                        cursorZoom = true;
                    else
                        cursorZoom = false;
                  });

          m_window.cursorCommand() = [&](auto const &event) {
              if (cursorLocked) {
                  if (cursorZoom) {
                      float deltaY = (cursorPosition.y - event.y);
                      float percY = deltaY / m_window.height();
                      m_camera.zoom(-percY * 50.f);
                  } else {
                      float deltaX = cursorPosition.x - event.x;
                      float deltaY = cursorPosition.y - event.y;
                      float percX = deltaX / m_window.width();
                      float percY = deltaY / m_window.height();
                      if (!yLock) m_camera.rotateAroundXPercent(percX);
                      if (!xLock) m_camera.rotateAroundYPercent(percY);
                  }
              }
              cursorPosition = event;
          };

        }

    private:
        io::Window &m_window;
        CameraT &m_camera;
        io::CursorPosition cursorPosition;
        bool cursorLocked = false;
        bool cursorZoom = false;
        bool xLock = false;
        bool yLock = false;
}; // end turntable camera controls
