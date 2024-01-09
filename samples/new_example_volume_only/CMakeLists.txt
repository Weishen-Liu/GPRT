# MIT License

# Copyright (c) 2022 Nathan V. Morrical

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

embed_devicecode(
  OUTPUT_TARGET
  new_example_volume_only_deviceCode
  ENTRY_POINTS
    simpleRayGen
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/shareDeviceCode.hlsl
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.hlsl
)

add_executable(new_example_volume_only  hostCode.cpp common/viewer.cpp common/configureImgui.cpp common/processVulkanResources.cpp common/jsonLoader.cpp)
target_include_directories(new_example_volume_only  PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(new_example_volume_only  PUBLIC ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(new_example_volume_only 
  PRIVATE
  new_example_volume_only_deviceCode
    gprt::gprt
    glfw
    tiny_obj_loader
    stb_image
    ${OPENGL_gl_LIBRARY}
    stdc++fs
    tfnmodule
    gdt
)
