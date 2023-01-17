// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

#ifndef INCLUDE_GPRT
#define INCLUDE_GPRT
// public GPRT API
#include <gprt.h>
#endif

#define LOG(message)                                             \
    std::cout << GPRT_TERMINAL_BLUE;                             \
    std::cout << "#gprt.sample(main): " << message << std::endl; \
    std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                          \
    std::cout << GPRT_TERMINAL_LIGHT_BLUE;                       \
    std::cout << "#gprt.sample(main): " << message << std::endl; \
    std::cout << GPRT_TERMINAL_DEFAULT;

#ifndef INCLUDE_VIEWER
#define INCLUDE_VIEWER
#include "./common/viewer.cpp"
#endif

#include <iostream>

extern GPRTProgram new_example_deviceCode;

int main(int ac, char **av)
{
    LOG("gprt example '" << av[0] << "' starting up");

    Viewer viewer;
    viewer.init(new_example_deviceCode);

    LOG("launching ...");
    viewer.run();

    // ##################################################################
    // and finally, clean up
    // ##################################################################

    LOG("cleaning up ...");
    viewer.destory();

    LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
