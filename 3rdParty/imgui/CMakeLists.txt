add_library(imgui-1 STATIC
  imgui.cpp
  imgui_draw.cpp
  imgui_demo.cpp
  imgui_tables.cpp
  imgui_widgets.cpp
  imgui_impl_glfw.cpp
  imgui_impl_opengl2.cpp
)

target_include_directories(imgui-1 PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../..)
target_include_directories(imgui-1 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/..)

target_link_libraries(imgui-1 PUBLIC glfw gprt::gprt)
target_compile_definitions(imgui-1 PUBLIC -DImTextureID=ImU64)