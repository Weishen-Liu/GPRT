add_library(viewer
  viewer.hpp
  viewer.cpp
)
target_include_directories(viewer PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(viewer PUBLIC ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(viewer PUBLIC glfw gprt::gprt tiny_obj_loader imgui-1)
