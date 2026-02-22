project(3dof_control)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(main
    main.cpp
    mouse.cpp
    mouse.h
    inverse_kinematics.cpp
    inverse_kinematics.h
)

target_link_libraries(main PRIVATE
        mujoco::mujoco
        glfw3
)

if(APPLE)
    target_link_libraries(main PRIVATE
            "-framework Cocoa"
            "-framework IOKit"
            "-framework OpenGL"
    )
elseif(WIN32)
    target_link_libraries(main PRIVATE opengl32)
elseif(UNIX AND NOT APPLE)
    target_link_libraries(main PRIVATE GL X11)
endif()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_compile_definitions(main PRIVATE
        SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
