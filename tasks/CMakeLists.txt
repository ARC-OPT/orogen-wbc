# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})

include(wbcTaskLib)
ADD_LIBRARY(${WBC_TASKLIB_NAME} SHARED 
    ${WBC_TASKLIB_SOURCES} ConstraintInterface.cpp KinematicModelInterface.cpp)
add_dependencies(${WBC_TASKLIB_NAME}
    regen-typekit)
include_directories(${PROJECT_SOURCE_DIR}/tasks)

TARGET_LINK_LIBRARIES(${WBC_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${WBC_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${WBC_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${WBC_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${WBC_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${WBC_TASKLIB_HEADERS} ConstraintInterface.hpp KinematicModelInterface.hpp
    DESTINATION include/orocos/wbc)

