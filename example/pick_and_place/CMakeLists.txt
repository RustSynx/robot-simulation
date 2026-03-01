project(pick_and_place)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(pick_and_place
    main.cpp
    mouse.cpp
    mouse.h
    inverse_kinematics.cpp
    inverse_kinematics.h
)

target_link_libraries(pick_and_place PRIVATE
        mujoco::mujoco
        glfw
)

if(APPLE)
    target_link_libraries(pick_and_place PRIVATE
            "-framework Cocoa"
            "-framework IOKit"
            "-framework OpenGL"
    )
elseif(WIN32)
    target_link_libraries(pick_and_place PRIVATE opengl32)
elseif(UNIX AND NOT APPLE)
    target_link_libraries(pick_and_place PRIVATE GL X11)
endif()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_compile_definitions(pick_and_place PRIVATE
        SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
