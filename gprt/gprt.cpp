// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <assert.h>
#include <climits>
#include <fstream>
#include <gprt_host.h>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>

#include <regex>

#ifdef __GNUC__
#include <execinfo.h>
#include <signal.h>
#include <sys/time.h>

#endif

#ifdef _WIN32
#define NOMINMAX
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#endif

// library for windowing
#include <GLFW/glfw3.h>

// library for image output
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

// For user interface
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

// For advanced vulkan memory allocation
#define VMA_VULKAN_VERSION 1002000   // Vulkan 1.2
#define VMA_IMPLEMENTATION
#include "vma/vk_mem_alloc.h"

// For FFX radix sort
#include "sort.h"

/** @brief A collection of features that are requested to support before
 * creating a GPRT context. These features might not be available on all
 * platforms.
 */
static struct RequestedFeatures {
  /** A window (VK_KHR_SURFACE, SWAPCHAIN, etc...)*/
  bool window = false;
  struct Window {
    uint32_t initialWidth;
    uint32_t initialHeight;
    std::string title;
  } windowProperties;

  uint32_t numRayTypes = 1;

  /** Ray queries enable inline ray tracing.
   * Not supported by some platforms like the A100, so requesting is important. */
  bool rayQueries = false;

  /*! returns whether logging is enabled */
  inline static bool logging() {
#ifdef NDEBUG
    return false;
#else
    return true;
#endif
  }
} requestedFeatures;

#if defined(_MSC_VER)
//&& !defined(__PRETTY_FUNCTION__)
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

// For error handling
namespace detail {
inline static std::string
backtrace() {
#ifdef __GNUC__
  static const int max_frames = 16;

  void *buffer[max_frames] = {0};
  int cnt = ::backtrace(buffer, max_frames);

  char **symbols = backtrace_symbols(buffer, cnt);

  if (symbols) {
    std::stringstream str;
    for (int n = 1; n < cnt; ++n)   // skip the 1st entry (address of this function)
    {
      str << symbols[n] << '\n';
    }
    free(symbols);
    return str.str();
  }
  return "";
#else
  return "not implemented yet";
#endif
}

inline void
gprtRaise_impl(std::string str) {
  fprintf(stderr, "%s\n", str.c_str());
#ifdef WIN32
  if (IsDebuggerPresent())
    DebugBreak();
  else
    assert(false);
#else
#ifndef NDEBUG
  std::string bt = ::detail::backtrace();
  fprintf(stderr, "%s\n", bt.c_str());
#endif
  raise(SIGINT);
#endif
}
}   // namespace detail

#define GPRT_RAISE(MSG) ::detail::gprtRaise_impl(MSG);

#define GPRT_NOTIMPLEMENTED                                                                                            \
  {                                                                                                                    \
    std::cerr << std::string(__PRETTY_FUNCTION__) << " not implemented" << std::endl;                                  \
    assert(false);                                                                                                     \
  };

#if 1
#define LOG_API_CALL() /* ignore */
#else
#define LOG_API_CALL() std::cout << "% " << __FUNCTION__ << "(...)" << std::endl;
#endif

#define LOG_INFO(message)                                                                                              \
  if (RequestedFeatures::logging())                                                                                    \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE << "#gprt info:  " << message << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_WARNING(message)                                                                                           \
  if (RequestedFeatures::logging())                                                                                    \
  std::cout << GPRT_TERMINAL_YELLOW << "#gprt warn:  " << message << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_ERROR(message)                                                                                             \
  {                                                                                                                    \
    if (RequestedFeatures::logging())                                                                                  \
      std::cout << GPRT_TERMINAL_RED << "#gprt error: " << message << GPRT_TERMINAL_DEFAULT << std::endl;              \
    GPRT_RAISE(message)                                                                                                \
  }

std::string
errorString(VkResult errorCode) {
  switch (errorCode) {
#define STR(r)                                                                                                         \
  case VK_##r:                                                                                                         \
    return #r
    STR(NOT_READY);
    STR(TIMEOUT);
    STR(EVENT_SET);
    STR(EVENT_RESET);
    STR(INCOMPLETE);
    STR(ERROR_OUT_OF_HOST_MEMORY);
    STR(ERROR_OUT_OF_DEVICE_MEMORY);
    STR(ERROR_INITIALIZATION_FAILED);
    STR(ERROR_DEVICE_LOST);
    STR(ERROR_MEMORY_MAP_FAILED);
    STR(ERROR_LAYER_NOT_PRESENT);
    STR(ERROR_EXTENSION_NOT_PRESENT);
    STR(ERROR_FEATURE_NOT_PRESENT);
    STR(ERROR_INCOMPATIBLE_DRIVER);
    STR(ERROR_TOO_MANY_OBJECTS);
    STR(ERROR_FORMAT_NOT_SUPPORTED);
    STR(ERROR_SURFACE_LOST_KHR);
    STR(ERROR_NATIVE_WINDOW_IN_USE_KHR);
    STR(SUBOPTIMAL_KHR);
    STR(ERROR_OUT_OF_DATE_KHR);
    STR(ERROR_INCOMPATIBLE_DISPLAY_KHR);
    STR(ERROR_VALIDATION_FAILED_EXT);
    STR(ERROR_INVALID_SHADER_NV);
    STR(ERROR_OUT_OF_POOL_MEMORY);
#undef STR
  default:
    return "UNKNOWN_ERROR";
  }
}

#define VK_CHECK_RESULT(f)                                                                                             \
  {                                                                                                                    \
    VkResult res = (f);                                                                                                \
    if (res != VK_SUCCESS) {                                                                                           \
      std::cout << "Fatal : VkResult is \"" << errorString(res) << "\" in " << __FILE__ << " at line " << __LINE__     \
                << "\n";                                                                                               \
      assert(res == VK_SUCCESS);                                                                                       \
    }                                                                                                                  \
  }

VKAPI_ATTR VkBool32 VKAPI_CALL
debugUtilsMessengerCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                            VkDebugUtilsMessageTypeFlagsEXT messageType,
                            const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData) {
  // Select prefix depending on flags passed to the callback
  std::string prefix("");

  if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
    prefix = "VERBOSE: ";
  } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
    prefix = "INFO: ";
  } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    prefix = "WARNING: ";
  } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    prefix = "ERROR: ";
  }

  // Display message to default output (console/logcat)
  std::stringstream debugMessage;
  // debugMessage << prefix << "[" << pCallbackData->messageIdNumber << "][" <<
  // pCallbackData->pMessageIdName << "] : " << pCallbackData->pMessage;
  debugMessage << pCallbackData->pMessage;

#if defined(__ANDROID__)
  if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    LOGE("%s", debugMessage.str().c_str());
  } else {
    LOGD("%s", debugMessage.str().c_str());
  }
#else
  if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    std::cerr << debugMessage.str() << "\n";
  } else {
    std::cout << debugMessage.str() << "\n";
  }
  fflush(stdout);
#endif

  // The return value of this callback controls whether the Vulkan call that
  // caused the validation message will be aborted or not We return VK_FALSE as
  // we DON'T want Vulkan calls that cause a validation message to abort If you
  // instead want to have calls abort, pass in VK_TRUE and the function will
  // return VK_ERROR_VALIDATION_FAILED_EXT
  return VK_FALSE;
}

// Contains definitions for internal entry points
// (bounds programs, transform programs...)
extern std::map<std::string, std::vector<uint8_t>> gprtDeviceCode;

extern std::map<std::string, std::vector<uint8_t>> sortDeviceCode;

// forward declarations...
struct Geom;
struct GeomType;
struct TriangleGeom;
struct TriangleGeomType;
struct AABBGeom;
struct AABBGeomType;

namespace gprt {
PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddress;
PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructure;
PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructure;
PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizes;
PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddress;
PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructures;
PFN_vkCmdCopyAccelerationStructureKHR vkCmdCopyAccelerationStructure;
PFN_vkBuildAccelerationStructuresKHR vkBuildAccelerationStructures;
PFN_vkCopyAccelerationStructureKHR vkCopyAccelerationStructure;
PFN_vkCmdTraceRaysKHR vkCmdTraceRays;
PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandles;
PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelines;
PFN_vkCmdWriteAccelerationStructuresPropertiesKHR vkCmdWriteAccelerationStructuresProperties;

PFN_vkCreateDebugUtilsMessengerEXT vkCreateDebugUtilsMessengerEXT;
PFN_vkDestroyDebugUtilsMessengerEXT vkDestroyDebugUtilsMessengerEXT;
VkDebugUtilsMessengerEXT debugUtilsMessenger;

PFN_vkCreateDebugReportCallbackEXT vkCreateDebugReportCallbackEXT;
PFN_vkDestroyDebugReportCallbackEXT vkDestroyDebugReportCallbackEXT;
VkDebugReportCallbackEXT debugReportCallback;
}   // namespace gprt

struct Stage {
  // for copying transforms into the instance buffer
  // std::string fillInstanceDataEntryPoint = "gprtFillInstanceData";
  // VkPipelineLayout fillInstanceDataPipelineLayout;
  // VkShaderModule fillInstanceDataShaderModule;
  // VkPipeline fillInstanceDataPipeline;
  std::string entryPoint;
  VkPipelineLayout layout;
  VkShaderModule module;
  VkPipeline pipeline;
};

struct Module {
  // std::string program;
  GPRTProgram program;

  Module(GPRTProgram program) { this->program = program; }

  ~Module() {}

  std::vector<uint32_t> getBinary(std::string entryType) {
    size_t sizeOfProgram = program[entryType].size() - 1;   // program is null terminated.
    std::vector<uint32_t> finalProgram(sizeOfProgram / 4);
    memcpy(finalProgram.data(), program[entryType].data(), sizeOfProgram);
    return finalProgram;
  }
};

struct Buffer {
  static std::vector<Buffer *> buffers;

  VkDevice device;
  VmaAllocator allocator;
  VkPhysicalDeviceMemoryProperties memoryProperties;
  VkCommandBuffer commandBuffer;
  VkQueue queue;

  /** @brief Usage flags to be filled by external source at buffer creation */
  VkBufferUsageFlags usageFlags;

  bool hostVisible;

  VkBuffer buffer = VK_NULL_HANDLE;
  VmaAllocation allocation;
  VkDeviceAddress deviceAddress = 0;

  // Some operations require buffers to be in a separate array.
  // Instead, we make our own virtual "sampler address space".
  uint32_t virtualAddress = -1;

  struct StagingBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VmaAllocation allocation;
  } stagingBuffer;

  VkDeviceSize size = 0;
  VkDeviceSize alignment = 16;
  void *mapped = nullptr;

  VkResult map(VkDeviceSize mapSize = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
    if (mapped)
      return VK_SUCCESS;

    if (hostVisible) {
      vmaInvalidateAllocation(allocator, allocation, 0, VK_WHOLE_SIZE);
      return vmaMapMemory(allocator, allocation, &mapped);
    } else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for buffer map! : \n" + errorString(err));

      // To do, consider allowing users to specify offsets here...
      VkBufferCopy region;
      region.srcOffset = 0;
      region.dstOffset = 0;
      region.size = size;
      vkCmdCopyBuffer(commandBuffer, buffer, stagingBuffer.buffer, 1, &region);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;
      submitInfo.pWaitDstStageMask = nullptr;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for buffer map! : \n" + errorString(err));

      vmaInvalidateAllocation(allocator, stagingBuffer.allocation, 0, VK_WHOLE_SIZE);
      return vmaMapMemory(allocator, stagingBuffer.allocation, &mapped);
    }
  }

  void unmap() {
    if (!mapped)
      return;

    if (hostVisible) {
      if (mapped) {
        vmaFlushAllocation(allocator, allocation, 0, VK_WHOLE_SIZE);
        vmaUnmapMemory(allocator, allocation);
        mapped = nullptr;
      }
    } else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for buffer map! : \n" + errorString(err));

      // To do, consider allowing users to specify offsets here...
      VkBufferCopy region;
      region.srcOffset = 0;
      region.dstOffset = 0;
      region.size = size;
      vkCmdCopyBuffer(commandBuffer, stagingBuffer.buffer, buffer, 1, &region);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;
      submitInfo.pWaitDstStageMask = nullptr;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for buffer map! : \n" + errorString(err));

      vmaFlushAllocation(allocator, stagingBuffer.allocation, 0, VK_WHOLE_SIZE);
      vmaUnmapMemory(allocator, stagingBuffer.allocation);
      mapped = nullptr;
    }
  }

  // flushes from host to device
  void flush() {
    if (hostVisible) {
      vmaFlushAllocation(allocator, allocation, 0, VK_WHOLE_SIZE);
    } else {
      vmaFlushAllocation(allocator, stagingBuffer.allocation, 0, VK_WHOLE_SIZE);
    }
  }

  // invalidates from device back to host
  void invalidate() {
    // device never directly writes to staging buffer
    vmaInvalidateAllocation(allocator, allocation, 0, VK_WHOLE_SIZE);
  }

  VkDeviceAddress getDeviceAddress() {
    VkBufferDeviceAddressInfoKHR info = {};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO_KHR;
    info.buffer = buffer;
    VkDeviceAddress addr = gprt::vkGetBufferDeviceAddress(device, &info);
    return addr;
  }

  /*! Calls vkDestroy on the buffer, and frees underlying memory */
  void destroy() {
    // Free sampler slot for use by subsequently made buffers
    Buffer::buffers[virtualAddress] = nullptr;

    unmap();

    if (buffer) {
      vmaDestroyBuffer(allocator, buffer, allocation);
      // vkDestroyBuffer(device, buffer, nullptr);
      buffer = VK_NULL_HANDLE;
    }
    if (stagingBuffer.buffer) {
      vmaDestroyBuffer(allocator, stagingBuffer.buffer, stagingBuffer.allocation);
      // vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
      // vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
      stagingBuffer.buffer = VK_NULL_HANDLE;
    }
  }

  /* Sets all bytes to 0 */
  void clear() {
    if (hostVisible) {
      if (!mapped)
        map();
      memset(mapped, 0, size);
    } else {
      map();
      memset(mapped, 0, size);
      unmap();
    }
  }

  void resize(size_t bytes, bool preserveContents) {
    // If the size is already okay, do nothing
    if (size == bytes)
      return;

    if (hostVisible) {
      // if we are host visible, we need to create a new buffer before releasing the
      // previous one to preserve values...

      if (preserveContents) {
        // Create the new buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;

        VkBuffer newBuffer;
        VmaAllocation newAllocation;
        VK_CHECK_RESULT(vmaCreateBuffer(allocator, &bufferCreateInfo, &allocInfo, &newBuffer, &newAllocation, nullptr));

        // Copy contents from old to new
        VkResult err;
        VkCommandBufferBeginInfo cmdBufInfo{};
        cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
        if (err)
          LOG_ERROR("failed to begin command buffer for buffer resize! : \n" + errorString(err));

        VkBufferCopy region;
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = std::min(size, VkDeviceSize(bytes));
        vkCmdCopyBuffer(commandBuffer, buffer, newBuffer, 1, &region);

        err = vkEndCommandBuffer(commandBuffer);
        if (err)
          LOG_ERROR("failed to end command buffer for buffer resize! : \n" + errorString(err));

        VkSubmitInfo submitInfo;
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.pNext = NULL;
        submitInfo.waitSemaphoreCount = 0;
        submitInfo.pWaitSemaphores = nullptr;
        submitInfo.pWaitDstStageMask = nullptr;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        submitInfo.signalSemaphoreCount = 0;
        submitInfo.pSignalSemaphores = nullptr;

        err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
        if (err)
          LOG_ERROR("failed to submit to queue for buffer resize! : \n" + errorString(err));

        err = vkQueueWaitIdle(queue);
        if (err)
          LOG_ERROR("failed to wait for queue idle for buffer resize! : \n" + errorString(err));

        // Free old buffer
        vmaUnmapMemory(allocator, allocation);
        vmaDestroyBuffer(allocator, buffer, allocation);

        // Assign new buffer
        buffer = newBuffer;
        allocation = newAllocation;
        size = bytes;
        deviceAddress = getDeviceAddress();
        vmaInvalidateAllocation(allocator, allocation, 0, VK_WHOLE_SIZE);
        vmaMapMemory(allocator, allocation, &mapped);
      } else {
        // Not preserving contents, we can delete old host buffer before making new one

        // Free old buffer
        vmaUnmapMemory(allocator, allocation);
        vmaDestroyBuffer(allocator, buffer, allocation);

        // Create the new buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;

        VK_CHECK_RESULT(vmaCreateBuffer(allocator, &bufferCreateInfo, &allocInfo, &buffer, &allocation, nullptr));
        vmaMapMemory(allocator, allocation, &mapped);
        size = bytes;
        deviceAddress = getDeviceAddress();
      }

    } else {
      // if we are device visible, we can use the staging buffer to temporarily hold previous values, and free
      // the old buffer before creating a new buffer.
      vmaInvalidateAllocation(allocator, allocation, 0, VK_WHOLE_SIZE);

      // Copy existing contents into staging buffer
      if (preserveContents) {
        VkResult err;
        VkCommandBufferBeginInfo cmdBufInfo{};
        cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
        if (err)
          LOG_ERROR("failed to begin command buffer for buffer resize! : \n" + errorString(err));

        VkBufferCopy region;
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = std::min(size, VkDeviceSize(bytes));
        vkCmdCopyBuffer(commandBuffer, buffer, stagingBuffer.buffer, 1, &region);

        err = vkEndCommandBuffer(commandBuffer);
        if (err)
          LOG_ERROR("failed to end command buffer for buffer resize! : \n" + errorString(err));

        VkSubmitInfo submitInfo;
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.pNext = NULL;
        submitInfo.waitSemaphoreCount = 0;
        submitInfo.pWaitSemaphores = nullptr;
        submitInfo.pWaitDstStageMask = nullptr;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        submitInfo.signalSemaphoreCount = 0;
        submitInfo.pSignalSemaphores = nullptr;

        err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
        if (err)
          LOG_ERROR("failed to submit to queue for buffer resize! : \n" + errorString(err));

        err = vkQueueWaitIdle(queue);
        if (err)
          LOG_ERROR("failed to wait for queue idle for buffer resize! : \n" + errorString(err));
      }

      // Release old device buffer
      vmaDestroyBuffer(allocator, buffer, allocation);

      {
        // Create the new buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        VK_CHECK_RESULT(vmaCreateBuffer(allocator, &bufferCreateInfo, &allocInfo, &buffer, &allocation, nullptr));
      }

      if (preserveContents) {
        // Copy contents from old staging buffer to new buffer
        VkResult err;
        VkCommandBufferBeginInfo cmdBufInfo{};
        cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
        if (err)
          LOG_ERROR("failed to begin command buffer for buffer resize! : \n" + errorString(err));

        VkBufferCopy region;
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = std::min(size, VkDeviceSize(bytes));
        vkCmdCopyBuffer(commandBuffer, stagingBuffer.buffer, buffer, 1, &region);

        err = vkEndCommandBuffer(commandBuffer);
        if (err)
          LOG_ERROR("failed to end command buffer for buffer resize! : \n" + errorString(err));

        VkSubmitInfo submitInfo;
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.pNext = NULL;
        submitInfo.waitSemaphoreCount = 0;
        submitInfo.pWaitSemaphores = nullptr;
        submitInfo.pWaitDstStageMask = nullptr;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        submitInfo.signalSemaphoreCount = 0;
        submitInfo.pSignalSemaphores = nullptr;

        err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
        if (err)
          LOG_ERROR("failed to submit to queue for buffer resize! : \n" + errorString(err));

        err = vkQueueWaitIdle(queue);
        if (err)
          LOG_ERROR("failed to wait for queue idle for buffer resize! : \n" + errorString(err));
      }

      size = bytes;
      deviceAddress = getDeviceAddress();

      // Release old staging buffer
      if (mapped) {
        vmaUnmapMemory(allocator, stagingBuffer.allocation);
      }
      vmaDestroyBuffer(allocator, stagingBuffer.buffer, stagingBuffer.allocation);
      {
        // Create the new staging buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
        VK_CHECK_RESULT(vmaCreateBuffer(allocator, &bufferCreateInfo, &allocInfo, &stagingBuffer.buffer,
                                        &stagingBuffer.allocation, nullptr));
      }

      // if buffer was previously mapped, restore mapped state
      if (mapped) {
        mapped = nullptr;
        map();
      }
    }
  }

  size_t getSize() { return (size_t) size; }

  /* Default Constructor */
  Buffer(){};

  ~Buffer(){};

  Buffer(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VmaAllocator _allocator,
         VkCommandBuffer _commandBuffer, VkQueue _queue, VkBufferUsageFlags _usageFlags,
         VkMemoryPropertyFlags _memoryPropertyFlags, VkDeviceSize _size, VkDeviceSize _alignment,
         void *data = nullptr) {

    // Hunt for an existing free virtual address for this buffer
    for (uint32_t i = 0; i < Buffer::buffers.size(); ++i) {
      if (Buffer::buffers[i] == nullptr) {
        Buffer::buffers[i] = this;
        virtualAddress = i;
        break;
      }
    }
    // If we cant find a free spot in the current buffer list, allocate a new
    // one
    if (virtualAddress == -1) {
      Buffer::buffers.push_back(this);
      virtualAddress = (uint32_t) buffers.size() - 1;
    }

    device = logicalDevice;
    allocator = _allocator;
    usageFlags = _usageFlags;
    size = _size;
    alignment = _alignment;
    commandBuffer = _commandBuffer;
    queue = _queue;

    // Check if the buffer can be mapped to a host pointer.
    // If the buffer isn't host visible, this is buffer and requires
    // an additional staging buffer...
    if ((_memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
      hostVisible = true;
    else
      hostVisible = false;

    // Create the buffer handle
    VkBufferCreateInfo bufferCreateInfo{};
    bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferCreateInfo.usage = usageFlags;
    bufferCreateInfo.size = size;

    VmaAllocationCreateInfo allocInfo = {};
    if (hostVisible) {
      allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
      allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    } else {
      allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    }

    VK_CHECK_RESULT(vmaCreateBufferWithAlignment(allocator, &bufferCreateInfo, &allocInfo, alignment, &buffer,
                                                 &allocation, nullptr));

    // VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &buffer));

    if (!hostVisible) {
      const VkBufferUsageFlags bufferUsageFlags =
          // means we can use this buffer to transfer into another
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
          // means we can use this buffer to receive data transferred from
          // another
          VK_BUFFER_USAGE_TRANSFER_DST_BIT;

      VkBufferCreateInfo bufferCreateInfo{};
      bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
      bufferCreateInfo.usage = bufferUsageFlags;
      bufferCreateInfo.size = size;
      // VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &stagingBuffer.buffer));

      VmaAllocationCreateInfo allocInfo = {};
      allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
      allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
      VK_CHECK_RESULT(vmaCreateBufferWithAlignment(allocator, &bufferCreateInfo, &allocInfo, alignment,
                                                   &stagingBuffer.buffer, &stagingBuffer.allocation, nullptr));
    }

    // If a pointer to the buffer data has been passed, map the buffer and
    // copy over the data
    if (data != nullptr) {
      map();
      memcpy(mapped, data, size);
      unmap();
    }

    // means we can get this buffer's address with vkGetBufferDeviceAddress
    if ((usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) != 0)
      deviceAddress = getDeviceAddress();
  }
};

std::vector<Buffer *> Buffer::buffers;

inline size_t
gprtFormatGetSize(GPRTFormat format) {
  switch (format) {
  case GPRT_FORMAT_R8_UINT:
    return 1;
  case GPRT_FORMAT_R8G8B8A8_UNORM:
    return 4;
  case GPRT_FORMAT_R8G8B8A8_SRGB:
    return 4;
  case GPRT_FORMAT_R32_SFLOAT:
    return 4;
  case GPRT_FORMAT_D32_SFLOAT:
    return 4;
  case GPRT_FORMAT_R32G32B32A32_SFLOAT:
    return 16;
  default:
    throw std::runtime_error("Error, unhandled image format");
    return -1;
  }
}

struct Texture {
  static std::vector<Texture *> texture1Ds;
  static std::vector<Texture *> texture2Ds;
  static std::vector<Texture *> texture3Ds;

  VkDevice device;
  VkPhysicalDeviceMemoryProperties memoryProperties;
  VkCommandBuffer commandBuffer;
  VkQueue queue;

  /** @brief Usage flags to be filled by external source at image creation */
  VkImageUsageFlags usageFlags;

  /** @brief Memory property flags to be filled by external source at image
   * creation */
  VkMemoryPropertyFlags memoryPropertyFlags;

  bool hostVisible;

  VkImage image = VK_NULL_HANDLE;
  VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
  VkDeviceMemory memory = VK_NULL_HANDLE;

  // Technically, textures in vulkan don't support addresses.
  // Instead, we make our own virtual "texture address space".
  uint32_t address = -1;

  VkImageView imageView = VK_NULL_HANDLE;

  uint32_t mipLevels;

  VkImageType imageType;

  VkFormat format;
  VkImageAspectFlagBits aspectFlagBits;

  uint32_t width;
  uint32_t height;
  uint32_t depth;
  VkDeviceSize size = 0;
  VkDeviceSize alignment = 0;
  void *mapped = nullptr;

  VkSubresourceLayout subresourceLayout{};

  struct StagingBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
  } stagingBuffer;

  /* Default Constructor */
  Texture(){};

  ~Texture(){};

  void setImageLayout(VkCommandBuffer cmdbuffer, VkImage image, VkImageLayout oldImageLayout,
                      VkImageLayout newImageLayout, VkImageSubresourceRange subresourceRange,
                      VkPipelineStageFlags srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                      VkPipelineStageFlags dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT) {
    // Create an image barrier object
    VkImageMemoryBarrier imageMemoryBarrier{};
    imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;

    // If we're transferring which queue owns this image, we need to set these.
    imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    imageMemoryBarrier.oldLayout = oldImageLayout;
    imageMemoryBarrier.newLayout = newImageLayout;
    imageMemoryBarrier.image = image;
    imageMemoryBarrier.subresourceRange = subresourceRange;

    // Source layouts (old)
    // Source access mask controls actions that have to be finished on the old
    // layout before it will be transitioned to the new layout
    switch (oldImageLayout) {
    case VK_IMAGE_LAYOUT_UNDEFINED:
      // Image layout is undefined (or does not matter)
      // Only valid as initial layout
      // No flags required, listed only for completeness
      imageMemoryBarrier.srcAccessMask = 0;
      break;

    case VK_IMAGE_LAYOUT_PREINITIALIZED:
      // Image is preinitialized
      // Only valid as initial layout for linear images, preserves memory
      // contents Make sure host writes have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
      // Image is a color attachment
      // Make sure any writes to the color buffer have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
      // Image is a depth/stencil attachment
      // Make sure any writes to the depth/stencil buffer have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
      // Image is a transfer source
      // Make sure any reads from the image have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      break;

    case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
      // Image is a transfer destination
      // Make sure any writes to the image have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
      // Image is read by a shader
      // Make sure any shader reads from the image have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
      break;
    default:
      // Other source layouts aren't handled (yet)
      break;
    }

    // Target layouts (new)
    // Destination access mask controls the dependency for the new image layout
    switch (newImageLayout) {
    case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
      // Image will be used as a transfer destination
      // Make sure any writes to the image have been finished
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
      // Image will be used as a transfer source
      // Make sure any reads from the image have been finished
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      break;

    case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
      // Image will be used as a color attachment
      // Make sure any writes to the color buffer have been finished
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
      // Image layout will be used as a depth/stencil attachment
      // Make sure any writes to depth/stencil buffer have been finished
      imageMemoryBarrier.dstAccessMask =
          imageMemoryBarrier.dstAccessMask | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
      // Image will be read in a shader (sampler, input attachment)
      // Make sure any writes to the image have been finished
      if (imageMemoryBarrier.srcAccessMask == 0) {
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
      }
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
      break;
    default:
      // Other source layouts aren't handled (yet)
      break;
    }

    // Put barrier inside setup command buffer
    vkCmdPipelineBarrier(cmdbuffer, srcStageMask, dstStageMask, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
  }

  VkResult map(VkDeviceSize mapSize = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
    if (hostVisible) {
      // Assuming layout is general
      if (mapped)
        return VK_SUCCESS;
      else
        return vkMapMemory(device, memory, offset, size, 0, &mapped);
    } else {
      // Finding that this causes bugs on Intel ARC. It seems that
      // vkCmdCopyImageToBuffer doesn't respect vulkan fences. We don't need
      // this, at the moment textures can only be written to by the host... But
      // it might be worth filing a bug over...

      // Update: we actually do need this to work in order to download the contents
      // of rasterized images.

      // VkResult err;
      // VkCommandBufferBeginInfo cmdBufInfo{};
      // cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      // err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      // if (err) LOG_ERROR("failed to begin command buffer for texture map! :
      // \n" + errorString(err));

      // // transition device to a transfer source format
      // setImageLayout(commandBuffer, image, layout,
      // VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, {aspectFlagBits, 0,
      // mipLevels, 0, 1}); layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

      // VkBufferImageCopy copyRegion;
      // copyRegion.imageOffset.x = 0;
      // copyRegion.imageOffset.y = 0;
      // copyRegion.imageOffset.z = 0;
      // copyRegion.imageExtent.width = width;
      // copyRegion.imageExtent.height = height;
      // copyRegion.imageExtent.depth = depth;
      // copyRegion.bufferOffset = 0;
      // copyRegion.bufferRowLength = 0;
      // copyRegion.bufferImageHeight = 0;
      // copyRegion.imageSubresource.aspectMask = aspectFlagBits;
      // copyRegion.imageSubresource.baseArrayLayer = 0;
      // copyRegion.imageSubresource.layerCount = 1;
      // copyRegion.imageSubresource.mipLevel = 0;
      // vkCmdCopyImageToBuffer(commandBuffer, image, layout,
      // stagingBuffer.buffer, 1, &copyRegion);

      // // transition device to previous format
      // setImageLayout(commandBuffer, image, layout,
      // VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, {aspectFlagBits,
      // 0, mipLevels, 0, 1}); layout =
      // VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

      // err = vkEndCommandBuffer(commandBuffer);
      // if (err) LOG_ERROR("failed to end command buffer for texture map! :
      // \n" + errorString(err));

      // VkSubmitInfo submitInfo;
      // submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      // submitInfo.pNext = NULL;
      // submitInfo.waitSemaphoreCount = 0;
      // submitInfo.pWaitSemaphores =
      // nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
      // submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
      // submitInfo.commandBufferCount = 1;
      // submitInfo.pCommandBuffers = &commandBuffer;
      // submitInfo.signalSemaphoreCount = 0;
      // submitInfo.pSignalSemaphores =
      // nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]}; err =
      // vkQueueSubmit(queue, 1, &submitInfo, nullptr); if (err)
      // LOG_ERROR("failed to submit to queue for texture map! : \n" +
      // errorString(err));

      // err = vkQueueWaitIdle(queue);
      // if (err) LOG_ERROR("failed to wait for queue idle for texture map! :
      // \n" + errorString(err));

      // todo, transfer device data to host
      if (mapped)
        return VK_SUCCESS;
      // else return vkMapMemory(device, stagingImage.memory, offset, mapSize,
      // 0, &mapped);
      else
        return vkMapMemory(device, stagingBuffer.memory, offset, mapSize, 0, &mapped);
    }
  }

  void unmap() {
    if (hostVisible) {
      // assuming layout is general
      if (mapped) {
        vkUnmapMemory(device, memory);
        mapped = nullptr;
      }
    } else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for texture map! : \n" + errorString(err));

      // transition device to a transfer destination format
      setImageLayout(commandBuffer, image, layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                     {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
      layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

      // copy data
      VkBufferImageCopy copyRegion;
      copyRegion.imageOffset.x = 0;
      copyRegion.imageOffset.y = 0;
      copyRegion.imageOffset.z = 0;
      copyRegion.imageExtent.width = width;
      copyRegion.imageExtent.height = height;
      copyRegion.imageExtent.depth = depth;
      copyRegion.bufferOffset = 0;
      copyRegion.bufferRowLength = 0;
      copyRegion.bufferImageHeight = 0;
      copyRegion.imageSubresource.aspectMask = aspectFlagBits;
      copyRegion.imageSubresource.baseArrayLayer = 0;
      copyRegion.imageSubresource.layerCount = 1;
      copyRegion.imageSubresource.mipLevel = 0;
      vkCmdCopyBufferToImage(commandBuffer, stagingBuffer.buffer, image, layout, 1, &copyRegion);

      // transition device to an optimal device format
      setImageLayout(commandBuffer, image, layout, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
      layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for texture map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for texture map! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for texture map! : \n" + errorString(err));

      // todo, transfer device data to device
      if (mapped) {
        // vkUnmapMemory(device, stagingImage.memory);
        vkUnmapMemory(device, stagingBuffer.memory);
        mapped = nullptr;
      }
    }
  }

  void clear() {
    VkResult err;
    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for texture mipmap generation! : \n" + errorString(err));

    // Move to a destination optimal format
    setImageLayout(commandBuffer, image, layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                   {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});

    // clear the image
    if (aspectFlagBits == VK_IMAGE_ASPECT_DEPTH_BIT) {
      VkClearDepthStencilValue val;
      val.depth = 1.f;
      val.stencil = 0;
      VkImageSubresourceRange range = {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1};
      vkCmdClearDepthStencilImage(commandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &val, 1, &range);
    } else {
      VkClearColorValue val;
      val.float32[0] = val.float32[1] = val.float32[2] = val.float32[3] = 0.f;
      val.int32[0] = val.int32[1] = val.int32[2] = val.int32[3] = 0;
      val.uint32[0] = val.uint32[1] = val.uint32[2] = val.uint32[3] = 0;
      VkImageSubresourceRange range = {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1};
      vkCmdClearColorImage(commandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &val, 1, &range);
    }

    // Now go back to the previous layout
    setImageLayout(commandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layout,
                   {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for texture mipmap generation! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for texture mipmap generation! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for texture mipmap generation! : \n" + errorString(err));
  }

  void generateMipmap() {
    // do nothing if we don't have a mipmap to generate
    if (mipLevels == 1)
      return;

    // double check we have the right usage flags...
    // Shouldn't happen, but doesn't hurt to double check.
    if ((usageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) == 0)
      LOG_ERROR("image needs transfer src usage bit for texture mipmap "
                "generation! \n");

    if ((usageFlags & VK_IMAGE_USAGE_TRANSFER_DST_BIT) == 0)
      LOG_ERROR("image needs transfer dst usage bit for texture mipmap "
                "generation! \n");

    VkResult err;
    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for texture mipmap generation! : \n" + errorString(err));

    // transition device to a transfer destination format
    setImageLayout(commandBuffer, image, layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                   {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
    layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.image = image;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.subresourceRange.aspectMask = aspectFlagBits;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;
    barrier.subresourceRange.levelCount = 1;

    int32_t mipWidth = width;
    int32_t mipHeight = height;
    int32_t mipDepth = depth;

    // note, loop here starts at 1, not 0
    for (uint32_t i = 1; i < mipLevels; i++) {
      // just transitioning the layouts of individual mips
      barrier.subresourceRange.baseMipLevel = i - 1;
      barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
      barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

      // this will wait for level i-1 to be filled from either a
      // vkCmdCopyBufferToImae call, or the previous blit command
      vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr,
                           0, nullptr, 1, &barrier);

      // here, we specify the regions to use for the blit
      VkImageBlit blit{};
      blit.srcOffsets[0] = {0, 0, 0};
      blit.srcOffsets[1] = {mipWidth, mipHeight, mipDepth};
      blit.srcSubresource.aspectMask = aspectFlagBits;
      blit.srcSubresource.mipLevel = i - 1;
      blit.srcSubresource.baseArrayLayer = 0;
      blit.srcSubresource.layerCount = 1;
      blit.dstOffsets[0] = {0, 0, 0};
      blit.dstOffsets[1] = {mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1,
                            mipDepth > 1 ? mipDepth / 2 : 1};
      blit.dstSubresource.aspectMask = aspectFlagBits;
      blit.dstSubresource.mipLevel = i;
      blit.dstSubresource.baseArrayLayer = 0;
      blit.dstSubresource.layerCount = 1;

      // This blit will downsample the current mip layer into the one above.
      // It also transitions the image
      vkCmdBlitImage(commandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image,
                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

      // Now, transition the layer to VK_IMAGE_LAYOUT_READ_ONLY_OPTIMAL
      barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
      barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

      vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                           nullptr, 0, nullptr, 1, &barrier);

      // now, divide the current mip dimensions by two.
      // mip levels can't be smaller than 1 though.
      if (mipWidth > 1)
        mipWidth /= 2;
      if (mipHeight > 1)
        mipHeight /= 2;
      if (mipDepth > 1)
        mipDepth /= 2;
    }

    // at the very end, we need one more barrier to transition the lastmip level
    barrier.subresourceRange.baseMipLevel = mipLevels - 1;
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 0, nullptr, 1, &barrier);

    // Now, all layers in the mip chan have this layout
    layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for texture mipmap generation! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for texture mipmap generation! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for texture mipmap generation! : \n" + errorString(err));
  }

  Texture(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer _commandBuffer, VkQueue _queue,
          VkImageUsageFlags _usageFlags, VkMemoryPropertyFlags _memoryPropertyFlags, VkImageType type, VkFormat _format,
          uint32_t _width, uint32_t _height, uint32_t _depth, bool allocateMipmap, const void *data = nullptr) {

    std::vector<Texture *> &textures = (type == VK_IMAGE_TYPE_1D)   ? texture1Ds
                                       : (type == VK_IMAGE_TYPE_2D) ? texture2Ds
                                                                    :
                                                                    /*(type == VK_IMAGE_TYPE_3D) ?*/ texture3Ds;

    // Hunt for an existing free address for this texture
    for (uint32_t i = 0; i < textures.size(); ++i) {
      if (textures[i] == nullptr) {
        textures[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current texture list, allocate a new
    // one
    if (address == -1) {
      textures.push_back(this);
      address = (uint32_t) textures.size() - 1;
    }

    device = logicalDevice;
    memoryPropertyFlags = _memoryPropertyFlags;
    usageFlags = _usageFlags;
    commandBuffer = _commandBuffer;
    queue = _queue;
    width = _width;
    height = _height;
    depth = _depth;
    format = _format;
    size = width * height * depth * gprtFormatGetSize((GPRTFormat) format);
    imageType = type;

    aspectFlagBits = (format == VK_FORMAT_D32_SFLOAT) ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT;

    uint32_t largestDimension = std::max(std::max(width, height), depth);
    if (allocateMipmap) {
      // Compute total mip levels (each being half the previous)
      // floor here accounts for non-power-of-two textures.
      mipLevels = static_cast<uint32_t>(std::floor(std::log2(largestDimension))) + 1;
    } else {
      mipLevels = 1;
    }

    // Check if the image can be mapped to a host pointer.
    // If the image isn't host visible, this is image and requires
    // an additional staging image...
    if ((memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
      hostVisible = true;
    else
      hostVisible = false;

    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memoryProperties);

    auto getMemoryType = [this](uint32_t typeBits, VkMemoryPropertyFlags properties,
                                VkBool32 *memTypeFound = nullptr) -> uint32_t {
      // memory type bits is a bitmask and contains one bit set for every
      // supported memory type. Bit i is set if and only if the memory type i in
      // the memory properties is supported.
      for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; i++) {
        if ((typeBits & 1) == 1) {
          if ((memoryProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            if (memTypeFound) {
              *memTypeFound = true;
            }
            return i;
          }
        }
        typeBits >>= 1;
      }

      if (memTypeFound) {
        *memTypeFound = false;
        return 0;
      } else {
        LOG_ERROR("Could not find a matching memory type");
      }
      return -1;
    };

    // Create the image handle
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = type;
    imageInfo.extent.width = width;
    imageInfo.extent.height = height;
    imageInfo.extent.depth = depth;
    imageInfo.mipLevels = mipLevels;
    imageInfo.arrayLayers = 1;
    imageInfo.format = format;

    // can be VK_IMAGE_TILING_LINEAR or VK_IMAGE_TILING_OPTIMAL.
    // VK_IMAGE_TILING_LINEAR means texels are laid out in a row-major order
    // VK_IMAGE_TILING_OPTIMAL means texels are laid out in an order that is
    // implementation defined for optimal access LINEAR is required for reading
    // texels directly. This tiling cannot be changed.
    if (hostVisible)
      imageInfo.tiling = VK_IMAGE_TILING_LINEAR;
    else
      imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;

    // Can be either VK_IMAGE_LAYOUT_UNDEFINED or
    // VK_IMAGE_LAYOUT_PREINITIALIZED. Preinitialized is required if we're
    // uploading straight to texture as if it were
    //   a staging image. If we instead use a staging buffer, then this should
    //   be undefined
    if (hostVisible)
      imageInfo.initialLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
    else
      imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    imageInfo.usage = usageFlags;
    if (aspectFlagBits == VK_IMAGE_ASPECT_COLOR_BIT)
      imageInfo.usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (aspectFlagBits == VK_IMAGE_ASPECT_DEPTH_BIT)
      imageInfo.usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;

    // Since this image is oonly going to be used by graphics queues, we have
    // this set to exclusive.
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    // If this image were to be used with MSAA as an attachment, we'd set this
    // to something other than
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.flags = 0;   // Optional, but some options here for sparse images,
                           // like volumes with large sections of just air values.

    VK_CHECK_RESULT(vkCreateImage(logicalDevice, &imageInfo, nullptr, &image));

    if (!hostVisible) {
      const VkBufferUsageFlags bufferUsageFlags =
          // means we can use this buffer to transfer into another
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
          // means we can use this buffer to receive data transferred from
          // another
          VK_BUFFER_USAGE_TRANSFER_DST_BIT;

      VkBufferCreateInfo bufferCreateInfo{};
      bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
      bufferCreateInfo.usage = bufferUsageFlags;
      bufferCreateInfo.size = size;
      VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &stagingBuffer.buffer));
    }

    // Create the memory backing up the image handle
    VkMemoryRequirements memReqs;
    vkGetImageMemoryRequirements(logicalDevice, image, &memReqs);
    VkMemoryAllocateInfo memAllocInfo{};
    memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memAllocInfo.allocationSize = memReqs.size;
    // Find a memory type index that firts the properties of the image
    memAllocInfo.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);

    VK_CHECK_RESULT(vkAllocateMemory(logicalDevice, &memAllocInfo, nullptr, &memory));
    alignment = memReqs.alignment;

    // Attach the memory to the image object
    VK_CHECK_RESULT(vkBindImageMemory(logicalDevice, image, memory, 0));

    if (!hostVisible) {
      const VkMemoryPropertyFlags memoryPropertyFlags =
          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |   // mappable to host with
                                                  // vkMapMemory
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;   // means "flush" and
                                                  // "invalidate"  not needed

      // Create the memory backing up the staging buffer handle
      VkMemoryRequirements memReqs;
      vkGetBufferMemoryRequirements(logicalDevice, stagingBuffer.buffer, &memReqs);
      VkMemoryAllocateInfo memAllocInfo{};
      memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
      memAllocInfo.allocationSize = memReqs.size;
      // Find a memory type index that fits the properties of the buffer
      memAllocInfo.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);

      VK_CHECK_RESULT(vkAllocateMemory(logicalDevice, &memAllocInfo, nullptr, &stagingBuffer.memory));

      // Attach the memory to the buffer object
      VkResult err = vkBindBufferMemory(device, stagingBuffer.buffer, stagingBuffer.memory, /* offset */ 0);
      if (err)
        LOG_ERROR("failed to bind staging buffer memory! : \n" + errorString(err));
    }

    // We need to transition the image into a known layout
    {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for buffer map! : \n" + errorString(err));

      setImageLayout(commandBuffer, image, layout, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
      layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for buffer map! : \n" + errorString(err));
    }

    // Now we need an image view
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = image;
    // note, image type and image view type share common enum values up to a
    // point... todo, how to handle cube maps? 1d/2d arrays? cube map arrays?
    viewInfo.viewType = (VkImageViewType) type;
    viewInfo.format = format;
    viewInfo.subresourceRange.aspectMask = aspectFlagBits;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = mipLevels;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    VK_CHECK_RESULT(vkCreateImageView(logicalDevice, &viewInfo, nullptr, &imageView));

    if (hostVisible) {
      VkImageSubresource subRes = {};
      subRes.aspectMask = aspectFlagBits;
      subRes.mipLevel = 0;
      subRes.arrayLayer = 0;
      vkGetImageSubresourceLayout(logicalDevice, image, &subRes, &subresourceLayout);
    }

    if (data != nullptr) {
      map();
      memcpy(mapped, data, size);
      unmap();

      if (mipLevels > 1)
        generateMipmap();
    }
  }

  /*! Calls vkDestroy on the image, and frees underlying memory */
  void destroy() {
    // Free texture slot for use by subsequently made textures
    std::vector<Texture *> &textures = (imageType == VK_IMAGE_TYPE_1D) ? texture1Ds
                                       : (imageType == VK_IMAGE_TYPE_2D)
                                           ? texture2Ds
                                           :
                                           /*(imageType == VK_IMAGE_TYPE_3D) ?*/ texture3Ds;

    textures[address] = nullptr;

    if (imageView) {
      vkDestroyImageView(device, imageView, nullptr);
      imageView = VK_NULL_HANDLE;
    }
    if (image) {
      vkDestroyImage(device, image, nullptr);
      image = VK_NULL_HANDLE;
    }
    if (memory) {
      vkFreeMemory(device, memory, nullptr);
      memory = VK_NULL_HANDLE;
    }
    if (stagingBuffer.buffer) {
      vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
      stagingBuffer.buffer = VK_NULL_HANDLE;
    }
    if (stagingBuffer.memory) {
      vkFreeMemory(device, stagingBuffer.memory, nullptr);
      stagingBuffer.memory = VK_NULL_HANDLE;
    }
  }
};

std::vector<Texture *> Texture::texture1Ds;
std::vector<Texture *> Texture::texture2Ds;
std::vector<Texture *> Texture::texture3Ds;

struct Sampler {
  static std::vector<Sampler *> samplers;

  VkDevice device;
  VkSampler sampler;

  // Technically, samplers in vulkan don't support addresses.
  // Instead, we make our own virtual "sampler address space".
  uint32_t address = -1;

  Sampler(){};

  ~Sampler(){};

  Sampler(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkFilter magFilter, VkFilter minFilter,
          VkSamplerMipmapMode mipFilter, uint32_t anisotropy, VkSamplerAddressMode addressMode,
          VkBorderColor borderColor) {
    // Hunt for an existing free address for this sampler
    for (uint32_t i = 0; i < Sampler::samplers.size(); ++i) {
      if (Sampler::samplers[i] == nullptr) {
        Sampler::samplers[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current sampler list, allocate a new
    // one
    if (address == -1) {
      samplers.push_back(this);
      address = (uint32_t) samplers.size() - 1;
    }

    device = logicalDevice;

    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = magFilter;
    samplerInfo.minFilter = minFilter;
    samplerInfo.addressModeU = addressMode;
    samplerInfo.addressModeV = addressMode;
    samplerInfo.addressModeW = addressMode;
    samplerInfo.anisotropyEnable = (anisotropy == 1) ? VK_FALSE : VK_TRUE;

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physicalDevice, &properties);
    samplerInfo.maxAnisotropy = std::min(properties.limits.maxSamplerAnisotropy, float(anisotropy));
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
    samplerInfo.mipmapMode = mipFilter;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = VK_LOD_CLAMP_NONE;
    samplerInfo.borderColor = borderColor;

    VK_CHECK_RESULT(vkCreateSampler(logicalDevice, &samplerInfo, nullptr, &sampler));
  }

  void destroy() {
    // Free sampler slot for use by subsequently made samplers
    Sampler::samplers[address] = nullptr;

    if (sampler) {
      vkDestroySampler(device, sampler, nullptr);
    }
  }
};

std::vector<Sampler *> Sampler::samplers;

struct SBTEntry {
  size_t recordSize = 0;
  uint8_t *SBTRecord = nullptr;
};

// At the moment, we actually just use ray generation programs for compute
// kernels. We do this instead of using actual Vulkan compute programs so that
// we can recycle the SBT records mechanism for compute IO, without
// introducing VK descriptor sets.
struct Compute : public SBTEntry {

  // Our own virtual "compute address space".
  uint32_t address = -1;
  static std::vector<Compute *> computes;

  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  VkDevice logicalDevice;
  std::string entryPoint;

  VkShaderModule shaderModule = VK_NULL_HANDLE;
  VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
  VkPipeline pipeline = VK_NULL_HANDLE;

  Compute(VkDevice _logicalDevice, Module *module, const char *_entryPoint, size_t recordSize) : SBTEntry() {
    // Hunt for an existing free address for this compute kernel
    for (uint32_t i = 0; i < Compute::computes.size(); ++i) {
      if (Compute::computes[i] == nullptr) {
        Compute::computes[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      Compute::computes.push_back(this);
      address = (uint32_t) Compute::computes.size() - 1;
    }

    entryPoint = std::string("__compute__") + std::string(_entryPoint);
    auto binary = module->getBinary("COMPUTE");

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);

    this->recordSize = recordSize;
    this->SBTRecord = (uint8_t *) malloc(recordSize);
  }
  ~Compute() {}

  void buildPipeline(VkDescriptorSetLayout samplerDescriptorSetLayout,
                     VkDescriptorSetLayout texture1DDescriptorSetLayout,
                     VkDescriptorSetLayout texture2DDescriptorSetLayout,
                     VkDescriptorSetLayout texture3DDescriptorSetLayout,
                     VkDescriptorSetLayout bufferDescriptorSetLayout, VkDescriptorSetLayout recordDescriptorSetLayout) {
    // If we already have a pipeline layout, free it so that we can make a new one
    if (pipelineLayout) {
      vkDestroyPipelineLayout(logicalDevice, pipelineLayout, nullptr);
      pipelineLayout = VK_NULL_HANDLE;
    }

    // If we already have a pipeline, free it so that we can make a new one
    if (pipeline) {
      vkDestroyPipeline(logicalDevice, pipeline, nullptr);
      pipeline = VK_NULL_HANDLE;
    }

    // currently not using cache.
    VkPipelineCache cache = VK_NULL_HANDLE;

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.size = 128;
    pushConstantRange.offset = 0;
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
    pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    std::vector<VkDescriptorSetLayout> layouts = {samplerDescriptorSetLayout,   texture1DDescriptorSetLayout,
                                                  texture2DDescriptorSetLayout, texture3DDescriptorSetLayout,
                                                  bufferDescriptorSetLayout,    recordDescriptorSetLayout};
    pipelineLayoutCreateInfo.setLayoutCount = (uint32_t) layouts.size();
    pipelineLayoutCreateInfo.pSetLayouts = layouts.data();
    pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(logicalDevice, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
      throw std::runtime_error("failed to create pipeline layout!");
    }

    VkComputePipelineCreateInfo computePipelineCreateInfo = {};
    computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    computePipelineCreateInfo.layout = pipelineLayout;
    computePipelineCreateInfo.flags = 0;
    computePipelineCreateInfo.stage = shaderStage;

    VkResult err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr, &pipeline);
    if (err) {
      LOG_ERROR("failed to create compute pipeline! Are all entrypoint names correct? \n" + errorString(err));
    }
  }

  void destroy() {
    // Free compute slot for use by subsequently made compute kernels
    Compute::computes[address] = nullptr;

    if (pipelineLayout) {
      vkDestroyPipelineLayout(logicalDevice, pipelineLayout, nullptr);
    }

    if (pipeline) {
      vkDestroyPipeline(logicalDevice, pipeline, nullptr);
    }

    if (shaderModule) {
      vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    }
  }
};

std::vector<Compute *> Compute::computes;

struct RayGen : public SBTEntry {
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  VkDevice logicalDevice;
  std::string entryPoint;

  RayGen(VkDevice _logicalDevice, Module *module, const char *_entryPoint, size_t recordSize) : SBTEntry() {
    entryPoint = std::string("__raygen__") + std::string(_entryPoint);
    auto binary = module->getBinary("RAYGEN");

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VkResult err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule);
    if (err)
      LOG_ERROR("failed to create shader module! : \n" + errorString(err));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);

    this->recordSize = recordSize;
    this->SBTRecord = (uint8_t *) malloc(recordSize);
  }
  ~RayGen() {}
  void destroy() {
    vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    free(this->SBTRecord);
  }
};

struct Miss : public SBTEntry {
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  VkDevice logicalDevice;
  std::string entryPoint;

  Miss(VkDevice _logicalDevice, Module *module, const char *_entryPoint, size_t recordSize) : SBTEntry() {
    entryPoint = std::string("__miss__") + std::string(_entryPoint);
    auto binary = module->getBinary("MISS");

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);

    this->recordSize = recordSize;
    this->SBTRecord = (uint8_t *) malloc(recordSize);
  }
  ~Miss() {}
  void destroy() {
    vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    free(this->SBTRecord);
  }
};

/* An abstraction for any sort of geometry type - describes the
    programs to use, and structure of the SBT records, when building
    shader binding tables (SBTs) with geometries of this type. This
    will later get subclassed into triangle geometries, user/custom
    primitive geometry types, etc */
struct GeomType : public SBTEntry {

  // Our own virtual "geom types address space".
  uint32_t address = -1;
  static std::vector<GeomType *> geomTypes;

  VkDevice logicalDevice;
  uint32_t numRayTypes;

  std::vector<VkPipelineShaderStageCreateInfo> closestHitShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> anyHitShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> intersectionShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> vertexShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> pixelShaderStages;

  std::vector<std::string> closestHitShaderEntryPoints;
  std::vector<std::string> anyHitShaderEntryPoints;
  std::vector<std::string> intersectionShaderEntryPoints;
  std::vector<std::string> vertexShaderEntryPoints;
  std::vector<std::string> pixelShaderEntryPoints;

  std::vector<bool> closestHitShaderUsed;
  std::vector<bool> intersectionShaderUsed;
  std::vector<bool> anyHitShaderUsed;
  std::vector<bool> vertexShaderUsed;
  std::vector<bool> pixelShaderUsed;

  // Optional resources for rasterizing geometry
  struct RasterData {
    uint32_t width = -1;
    uint32_t height = -1;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkFramebuffer frameBuffer = VK_NULL_HANDLE;
    Texture *colorAttachment = nullptr;
    Texture *depthAttachment = nullptr;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
  };
  std::vector<RasterData> raster;

  GeomType(VkDevice _logicalDevice, uint32_t numRayTypes, size_t recordSize) : SBTEntry() {
    // Hunt for an existing free address for this geom type
    for (uint32_t i = 0; i < GeomType::geomTypes.size(); ++i) {
      if (GeomType::geomTypes[i] == nullptr) {
        GeomType::geomTypes[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      GeomType::geomTypes.push_back(this);
      address = (uint32_t) GeomType::geomTypes.size() - 1;
    }

    this->numRayTypes = numRayTypes;
    closestHitShaderStages.resize(numRayTypes, {});
    anyHitShaderStages.resize(numRayTypes, {});
    intersectionShaderStages.resize(numRayTypes, {});
    vertexShaderStages.resize(numRayTypes, {});
    pixelShaderStages.resize(numRayTypes, {});

    closestHitShaderEntryPoints.resize(numRayTypes, {});
    anyHitShaderEntryPoints.resize(numRayTypes, {});
    intersectionShaderEntryPoints.resize(numRayTypes, {});
    vertexShaderEntryPoints.resize(numRayTypes, {});
    pixelShaderEntryPoints.resize(numRayTypes, {});

    closestHitShaderUsed.resize(numRayTypes, false);
    intersectionShaderUsed.resize(numRayTypes, false);
    anyHitShaderUsed.resize(numRayTypes, false);
    vertexShaderUsed.resize(numRayTypes, false);
    pixelShaderUsed.resize(numRayTypes, false);

    raster.resize(numRayTypes);

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    // store size, but don't allocate. Will be done by geom instances.
    this->recordSize = recordSize;
  }
  ~GeomType() {
    // vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
  }

  virtual GPRTGeomKind getKind() { return GPRT_UNKNOWN; }

  void setClosestHit(int rayType, Module *module, const char *entryPoint) {
    closestHitShaderUsed[rayType] = true;
    closestHitShaderEntryPoints[rayType] = std::string("__closesthit__") + std::string(entryPoint);
    auto binary = module->getBinary("CLOSESTHIT");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    closestHitShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    closestHitShaderStages[rayType].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    closestHitShaderStages[rayType].module = shaderModule;
    closestHitShaderStages[rayType].pName = closestHitShaderEntryPoints[rayType].c_str();
    assert(closestHitShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void setAnyHit(int rayType, Module *module, const char *entryPoint) {
    anyHitShaderUsed[rayType] = true;
    anyHitShaderEntryPoints[rayType] = std::string("__anyhit__") + std::string(entryPoint);
    auto binary = module->getBinary("ANYHIT");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    anyHitShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    anyHitShaderStages[rayType].stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    anyHitShaderStages[rayType].module = shaderModule;
    anyHitShaderStages[rayType].pName = anyHitShaderEntryPoints[rayType].c_str();
    assert(anyHitShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void setIntersection(int rayType, Module *module, const char *entryPoint) {
    intersectionShaderUsed[rayType] = true;
    intersectionShaderEntryPoints[rayType] = std::string("__intersection__") + std::string(entryPoint);
    auto binary = module->getBinary("INTERSECTION");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    intersectionShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    intersectionShaderStages[rayType].stage = VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
    intersectionShaderStages[rayType].module = shaderModule;
    intersectionShaderStages[rayType].pName = intersectionShaderEntryPoints[rayType].c_str();
    assert(intersectionShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void setVertex(int rasterType, Module *module, const char *entryPoint) {
    vertexShaderUsed[rasterType] = true;
    vertexShaderEntryPoints[rasterType] = std::string("__vertex__") + std::string(entryPoint);
    auto binary = module->getBinary("VERTEX");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    vertexShaderStages[rasterType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertexShaderStages[rasterType].stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexShaderStages[rasterType].module = shaderModule;
    vertexShaderStages[rasterType].pName = vertexShaderEntryPoints[rasterType].c_str();
    assert(vertexShaderStages[rasterType].module != VK_NULL_HANDLE);
  }

  void setPixel(int rasterType, Module *module, const char *entryPoint) {
    pixelShaderUsed[rasterType] = true;
    pixelShaderEntryPoints[rasterType] = std::string("__pixel__") + std::string(entryPoint);
    auto binary = module->getBinary("PIXEL");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    pixelShaderStages[rasterType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pixelShaderStages[rasterType].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    pixelShaderStages[rasterType].module = shaderModule;
    pixelShaderStages[rasterType].pName = pixelShaderEntryPoints[rasterType].c_str();
    assert(pixelShaderStages[rasterType].module != VK_NULL_HANDLE);
  }

  void setRasterAttachments(uint32_t rasterType, Texture *colorTexture, Texture *depthTexture) {
    if (colorTexture->width != depthTexture->width || colorTexture->height != depthTexture->height) {
      throw std::runtime_error("Error, color and depth attachment textures must have equal dimensions!");
    } else {
      raster[rasterType].width = colorTexture->width;
      raster[rasterType].height = colorTexture->height;
    }

    if (raster[rasterType].renderPass != VK_NULL_HANDLE)
      vkDestroyRenderPass(logicalDevice, raster[rasterType].renderPass, nullptr);

    raster[rasterType].colorAttachment = colorTexture;
    raster[rasterType].depthAttachment = depthTexture;

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = colorTexture->format;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    // clear here says to clear the values to a constant at start.
    // colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;   // DONT_CARE;
    // save rasterized fragments to memory
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    // not currently using a stencil
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    // Initial and final layouts of the texture
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = depthTexture->format;
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;   // VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

    std::vector<VkAttachmentDescription> attachments = {colorAttachment, depthAttachment};

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 1;
    depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;
    subpass.pDepthStencilAttachment = &depthAttachmentRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    createInfo.pNext = nullptr;
    createInfo.attachmentCount = (uint32_t) attachments.size();
    createInfo.pAttachments = attachments.data();
    createInfo.subpassCount = 1;
    createInfo.pSubpasses = &subpass;
    createInfo.dependencyCount = 1;
    createInfo.pDependencies = &dependency;

    vkCreateRenderPass(logicalDevice, &createInfo, nullptr, &raster[rasterType].renderPass);

    VkImageView attachmentViews[] = {raster[rasterType].colorAttachment->imageView,
                                     raster[rasterType].depthAttachment->imageView};

    VkFramebufferCreateInfo framebufferInfo{};
    framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    framebufferInfo.renderPass = raster[rasterType].renderPass;
    framebufferInfo.attachmentCount = 2;
    framebufferInfo.pAttachments = attachmentViews;
    framebufferInfo.width = raster[rasterType].width;
    framebufferInfo.height = raster[rasterType].height;
    framebufferInfo.layers = 1;

    if (vkCreateFramebuffer(logicalDevice, &framebufferInfo, nullptr, &raster[rasterType].frameBuffer) != VK_SUCCESS) {
      throw std::runtime_error("failed to create framebuffer!");
    }
  }

  void buildRasterPipeline(uint32_t rasterType, VkDescriptorSetLayout samplerDescriptorSetLayout,
                           VkDescriptorSetLayout texture1DDescriptorSetLayout,
                           VkDescriptorSetLayout texture2DDescriptorSetLayout,
                           VkDescriptorSetLayout texture3DDescriptorSetLayout,
                           VkDescriptorSetLayout bufferDescriptorSetLayout,
                           VkDescriptorSetLayout recordDescriptorSetLayout) {
    // we need both of these to be set, otherwise we can't build a raster pipeline...
    if (!vertexShaderUsed[rasterType] || !pixelShaderUsed[rasterType])
      return;

    // we also need a framebuffer...
    if (!raster[rasterType].frameBuffer)
      return;

    // If we already have a pipeline layout, free it so that we can make a new one
    if (raster[rasterType].pipelineLayout) {
      vkDestroyPipelineLayout(logicalDevice, raster[rasterType].pipelineLayout, nullptr);
      raster[rasterType].pipelineLayout = VK_NULL_HANDLE;
    }

    // If we already have a pipeline, free it so that we can make a new one
    if (raster[rasterType].pipeline) {
      vkDestroyPipeline(logicalDevice, raster[rasterType].pipeline, nullptr);
      raster[rasterType].pipeline = VK_NULL_HANDLE;
    }

    std::vector<VkPipelineShaderStageCreateInfo> shaderStages = {vertexShaderStages[rasterType],
                                                                 pixelShaderStages[rasterType]};

    // describes format of the vertex data
    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.vertexBindingDescriptionCount = 0;
    vertexInputInfo.pVertexBindingDescriptions = nullptr;   // Optional
    vertexInputInfo.vertexAttributeDescriptionCount = 0;
    vertexInputInfo.pVertexAttributeDescriptions = nullptr;   // Optional

    // describes what kind of geometry will be drawn
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    // describes the region of the framebuffer that the output will be rendered to
    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = (float) raster[rasterType].width;
    viewport.height = (float) raster[rasterType].height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    // used to potentially crop the image
    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent.width = raster[rasterType].width;
    scissor.extent.height = raster[rasterType].height;

    // Things that can change without needing to rebuild the pipeline...
    std::vector<VkDynamicState> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.pViewports = &viewport;
    viewportState.scissorCount = 1;
    viewportState.pScissors = &scissor;

    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages.data();

    // takes geometry and turns it into fragments
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
    rasterizer.depthBiasConstantFactor = 0.0f;   // Optional
    rasterizer.depthBiasClamp = 0.0f;            // Optional
    rasterizer.depthBiasSlopeFactor = 0.0f;      // Optional

    // for MSAA, one way to do antialiasing
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0f;            // Optional
    multisampling.pSampleMask = nullptr;              // Optional
    multisampling.alphaToCoverageEnable = VK_FALSE;   // Optional
    multisampling.alphaToOneEnable = VK_FALSE;        // Optional

    // compositing configuration
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_TRUE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;                   // Optional
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;   // Optional
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;                              // Optional
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;                   // Optional
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;   // Optional
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;                              // Optional

    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.logicOp = VK_LOGIC_OP_COPY;   // Optional
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;
    colorBlending.blendConstants[0] = 0.0f;   // Optional
    colorBlending.blendConstants[1] = 0.0f;   // Optional
    colorBlending.blendConstants[2] = 0.0f;   // Optional
    colorBlending.blendConstants[3] = 0.0f;   // Optional

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.minDepthBounds = 0.0f;   // Optional
    depthStencil.maxDepthBounds = 1.0f;   // Optional
    depthStencil.stencilTestEnable = VK_FALSE;
    depthStencil.front = {};   // Optional
    depthStencil.back = {};    // Optional

    // The layout here describes descriptor sets and push constants used
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    std::vector<VkDescriptorSetLayout> layouts = {samplerDescriptorSetLayout,   texture1DDescriptorSetLayout,
                                                  texture2DDescriptorSetLayout, texture3DDescriptorSetLayout,
                                                  bufferDescriptorSetLayout,    recordDescriptorSetLayout};
    pipelineLayoutInfo.setLayoutCount = (uint32_t) layouts.size();
    pipelineLayoutInfo.pSetLayouts = layouts.data();

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.size = 128;
    pushConstantRange.offset = 0;
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(logicalDevice, &pipelineLayoutInfo, nullptr, &raster[rasterType].pipelineLayout) !=
        VK_SUCCESS) {
      throw std::runtime_error("failed to create pipeline layout!");
    }

    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;   // Optional
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;

    pipelineInfo.layout = raster[rasterType].pipelineLayout;

    pipelineInfo.renderPass = raster[rasterType].renderPass;
    pipelineInfo.subpass = 0;

    pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;   // Optional
    pipelineInfo.basePipelineIndex = -1;                // Optional

    if (vkCreateGraphicsPipelines(logicalDevice, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr,
                                  &raster[rasterType].pipeline) != VK_SUCCESS) {
      throw std::runtime_error("failed to create graphics pipeline!");
    }
  }

  void destroy() {
    // Free geomtype slot for use by subsequently made geomtypes
    GeomType::geomTypes[address] = nullptr;

    for (uint32_t i = 0; i < closestHitShaderStages.size(); ++i) {
      if (closestHitShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, closestHitShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < anyHitShaderStages.size(); ++i) {
      if (anyHitShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, anyHitShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < intersectionShaderStages.size(); ++i) {
      if (intersectionShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, intersectionShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < vertexShaderStages.size(); ++i) {
      if (vertexShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, vertexShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < pixelShaderStages.size(); ++i) {
      if (pixelShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, pixelShaderStages[i].module, nullptr);
    }

    for (uint32_t i = 0; i < raster.size(); ++i) {
      if (raster[i].pipelineLayout) {
        vkDestroyPipelineLayout(logicalDevice, raster[i].pipelineLayout, nullptr);
      }

      if (raster[i].pipeline) {
        vkDestroyPipeline(logicalDevice, raster[i].pipeline, nullptr);
      }

      if (raster[i].renderPass) {
        vkDestroyRenderPass(logicalDevice, raster[i].renderPass, nullptr);
      }

      if (raster[i].frameBuffer) {
        vkDestroyFramebuffer(logicalDevice, raster[i].frameBuffer, nullptr);
      }
    }
  }

  virtual Geom *createGeom() { return nullptr; };
};

std::vector<GeomType *> GeomType::geomTypes;

/*! An actual geometry object with primitives - this class is still
  abstract, and will get fleshed out in its derived classes
  (AABBGeom, TriangleGeom, ...) */
struct Geom : public SBTEntry {
  // Our own virtual "geometry address space".
  VkDeviceAddress address = -1;

  static std::vector<Geom *> geoms;

  Geom() : SBTEntry() {
    // Hunt for an existing free address for this geometry
    for (uint32_t i = 0; i < Geom::geoms.size(); ++i) {
      if (Geom::geoms[i] == nullptr) {
        Geom::geoms[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      geoms.push_back(this);
      address = geoms.size() - 1;
    }
  };
  ~Geom(){};

  void destroy() {
    // Free geometry slot for use by subsequently made geometries
    Geom::geoms[address] = nullptr;
  }

  /*! This acts as a template that describes this geometry's variables and
      programs. */
  GeomType *geomType;
};

std::vector<Geom *> Geom::geoms;

struct TriangleGeom : public Geom {
  struct {
    uint32_t count = 0;         // number of indices
    uint32_t stride = 0;        // stride between indices
    uint32_t offset = 0;        // offset in bytes to the first index
    uint32_t firstVertex = 0;   // added to the index values before fetching vertices
    Buffer *buffer = nullptr;
  } index;

  struct {
    uint32_t count = 0;    // number of vertices
    uint32_t stride = 0;   // stride between vertices
    uint32_t offset = 0;   // an offset in bytes to the first vertex
    std::vector<Buffer *> buffers;
  } vertex;

  struct {
    size_t count = 0;    // number of vertices
    size_t stride = 0;   // stride between vertices
    size_t offset = 0;   // an offset in bytes to the first vertex
    std::vector<Buffer *> buffers;
  } normal;

  TriangleGeom(TriangleGeomType *_geomType) : Geom() {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~TriangleGeom() { free(this->SBTRecord); };

  void setVertices(Buffer *vertices, uint32_t count, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    vertex.buffers.resize(1);
    vertex.buffers[0] = vertices;
    vertex.count = count;
    vertex.stride = stride;
    vertex.offset = offset;
  }

  void setVertexNormal(Buffer *normals, size_t count, size_t stride, size_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    normal.buffers.resize(1);
    normal.buffers[0] = normals;
    normal.count = count;
    normal.stride = stride;
    normal.offset = offset;
  }

  void setIndices(Buffer *indices, uint32_t count, uint32_t stride, uint32_t offset) {
    index.buffer = indices;
    index.count = count;
    index.stride = stride;
    index.offset = offset;
  }
};

struct TriangleGeomType : public GeomType {
  TriangleGeomType(VkDevice logicalDevice, uint32_t numRayTypes, size_t recordSize)
      : GeomType(logicalDevice, numRayTypes, recordSize) {}
  ~TriangleGeomType() {}

  Geom *createGeom() { return new TriangleGeom(this); }

  GPRTGeomKind getKind() { return GPRT_TRIANGLES; }
};

struct AABBGeom : public Geom {
  struct {
    uint32_t count;
    uint32_t stride;
    uint32_t offset;
    std::vector<Buffer *> buffers;
  } aabb;

  AABBGeom(AABBGeomType *_geomType) : Geom() {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~AABBGeom() { free(this->SBTRecord); };

  void setAABBs(Buffer *aabbs, uint32_t count, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    aabb.buffers.resize(1);
    aabb.buffers[0] = aabbs;
    aabb.count = count;
    aabb.stride = stride;
    aabb.offset = offset;
  }
};

struct AABBGeomType : public GeomType {
  AABBGeomType(VkDevice _logicalDevice, uint32_t numRayTypes, size_t recordSize)
      : GeomType(_logicalDevice, numRayTypes, recordSize) {}
  ~AABBGeomType() {}
  Geom *createGeom() { return new AABBGeom(this); }

  GPRTGeomKind getKind() { return GPRT_AABBS; }
};

typedef enum {
  GPRT_UNKNOWN_ACCEL = 0x0,
  GPRT_INSTANCE_ACCEL = 0x1,
  GPRT_TRIANGLE_ACCEL = 0x2,
  GPRT_AABB_ACCEL = 0x3,
} AccelType;

struct Accel {
  VkPhysicalDevice physicalDevice;
  VkDevice logicalDevice;
  VmaAllocator allocator;
  VkCommandBuffer commandBuffer;
  VkQueue queue;
  VkDeviceAddress address = 0;
  VkAccelerationStructureKHR accelerationStructure = VK_NULL_HANDLE;
  VkAccelerationStructureKHR compactAccelerationStructure = VK_NULL_HANDLE;
  VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures;
  VkPhysicalDeviceAccelerationStructurePropertiesKHR accelerationStructureProperties;
  GPRTBuildMode buildMode = GPRT_BUILD_MODE_UNINITIALIZED;
  bool allowCompaction = false;
  bool minimizeMemory = false;
  bool isCompact = false;

  Buffer *accelBuffer = nullptr;
  Buffer *compactBuffer = nullptr;
  Buffer *scratchBuffer = nullptr;   // Can we make this static? That way, all trees could share the scratch...

  Accel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VmaAllocator allocator, VkCommandBuffer commandBuffer,
        VkQueue queue) {
    this->physicalDevice = physicalDevice;
    this->logicalDevice = logicalDevice;
    this->allocator = allocator;
    this->commandBuffer = commandBuffer;
    this->queue = queue;

    accelerationStructureFeatures = {};
    accelerationStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;

    VkPhysicalDeviceFeatures2 deviceFeatures2{};
    deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    deviceFeatures2.pNext = &accelerationStructureFeatures;

    vkGetPhysicalDeviceFeatures2(physicalDevice, &deviceFeatures2);

    accelerationStructureProperties = {};
    accelerationStructureProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;

    VkPhysicalDeviceProperties2 deviceProperties2{};
    deviceProperties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    deviceProperties2.pNext = &accelerationStructureProperties;

    vkGetPhysicalDeviceProperties2(physicalDevice, &deviceProperties2);
  };

  ~Accel(){};

  virtual void build(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes,
                     GPRTBuildMode mode, bool allowCompaction, bool minimizeMemory){};
  virtual void update(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes){};
  virtual void compact(VkQueryPool compactedSizeQueryPool){};
  virtual void destroy(){};
  virtual size_t getSize() { return -1; };
  virtual AccelType getType() { return GPRT_UNKNOWN_ACCEL; }
};

struct TriangleAccel : public Accel {
  std::vector<TriangleGeom *> geometries;

  // caching these for fast tree updates
  std::vector<VkAccelerationStructureBuildRangeInfoKHR> accelerationBuildStructureRangeInfos;
  std::vector<VkAccelerationStructureBuildRangeInfoKHR *> accelerationBuildStructureRangeInfoPtrs;
  std::vector<VkAccelerationStructureGeometryKHR> accelerationStructureGeometries;
  std::vector<uint32_t> maxPrimitiveCounts;

  TriangleAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VmaAllocator allocator,
                VkCommandBuffer commandBuffer, VkQueue queue, size_t numGeometries, TriangleGeom *geometries)
      : Accel(physicalDevice, logicalDevice, allocator, commandBuffer, queue) {
    this->geometries.resize(numGeometries);
    memcpy(this->geometries.data(), geometries, sizeof(GPRTGeom *) * numGeometries);
  };

  ~TriangleAccel(){};

  AccelType getType() { return GPRT_TRIANGLE_ACCEL; }

  size_t getSize() {
    size_t size = 0;
    if (accelBuffer)
      size += accelBuffer->getSize();
    if (compactBuffer)
      size += compactBuffer->getSize();
    if (scratchBuffer)
      size += scratchBuffer->getSize();
    return size;
  };

  void build(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes,
             GPRTBuildMode mode, bool allowCompaction, bool minimizeMemory) {
    VkResult err;

    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());
    maxPrimitiveCounts.resize(geometries.size());
    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
      //   means, anyhit shader is disabled

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...

      geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
      geom.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;

      // vertex data
      geom.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
      geom.geometry.triangles.vertexData.deviceAddress =
          geometries[gid]->vertex.buffers[0]->deviceAddress + geometries[gid]->vertex.offset;
      geom.geometry.triangles.vertexStride = geometries[gid]->vertex.stride;
      geom.geometry.triangles.maxVertex = geometries[gid]->vertex.count;

      // index data
      geom.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
      // note, offset accounted for in range
      geom.geometry.triangles.indexData.deviceAddress = geometries[gid]->index.buffer->deviceAddress;
      maxPrimitiveCounts[gid] = geometries[gid]->index.count;

      // transform data
      // note, offset accounted for in range
      geom.geometry.triangles.transformData.hostAddress = nullptr;
      // if the above is null, then that indicates identity

      auto &geomRange = accelerationBuildStructureRangeInfos[gid];
      accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
      geomRange.primitiveCount = geometries[gid]->index.count;
      geomRange.primitiveOffset = geometries[gid]->index.offset;
      geomRange.firstVertex = geometries[gid]->index.firstVertex;
      geomRange.transformOffset = 0;
    }

    // Get size info
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (mode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (mode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationStructureBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                               &accelerationStructureBuildGeometryInfo, maxPrimitiveCounts.data(),
                                               &accelerationStructureBuildSizesInfo);

    // If previously compacted, free those resources up.
    if (compactBuffer) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    // Destroy old accel handle
    if (accelBuffer && accelBuffer->size < accelerationStructureBuildSizesInfo.accelerationStructureSize) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    if (!accelBuffer) {
      accelBuffer =
          new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                     // means we can use this buffer as a means of storing an acceleration
                     // structure
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                         // means we can get this buffer's address with
                         // vkGetBufferDeviceAddress
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         // means we can use this buffer as a storage buffer
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     // means that this memory is stored directly on the device
                     //  (rather than the host, or in a special host/device section)
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.accelerationStructureSize,
                     accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(logicalDevice, &accelerationStructureCreateInfo, nullptr,
                                                &accelerationStructure);
      if (err)
        LOG_ERROR("failed to create acceleration structure for triangle accel "
                  "build! : \n" +
                  errorString(err));
    }

    if (scratchBuffer && scratchBuffer->size < accelerationStructureBuildSizesInfo.buildScratchSize) {
      scratchBuffer->destroy();
      delete (scratchBuffer);
      scratchBuffer = nullptr;
    }

    if (!scratchBuffer) {
      scratchBuffer =
          new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                     // means that the buffer can be used in a VkDescriptorBufferInfo. //
                     // Is this required? If not, remove this...
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                         // means we can get this buffer's address with
                         // vkGetBufferDeviceAddress
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         // means we can use this buffer as a storage buffer
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     // means that this memory is stored directly on the device
                     //  (rather than the host, or in a special host/device section)
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.buildScratchSize,
                     accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (mode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (mode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->deviceAddress;

    // Build the acceleration structure on the device via a one-time command
    // buffer submission Some implementations may support acceleration structure
    // building on the host
    // (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands),
    // but we prefer device builds VkCommandBuffer commandBuffer =
    // vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(commandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    // VkFenceCreateInfo fenceInfo {};
    // fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    // fenceInfo.flags = 0;
    // VkFence fence;
    // err = vkCreateFence(logicalDevice, &fenceInfo, nullptr, &fence);
    // if (err) LOG_ERROR("failed to create fence for triangle accel build! :
    // \n" + errorString(err));

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for triangle accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for triangle accel build! : \n" + errorString(err));

    // Wait for the fence to signal that command buffer has finished executing
    // err = vkWaitForFences(logicalDevice, 1, &fence, VK_TRUE, 100000000000
    // /*timeout*/); if (err) LOG_ERROR("failed to wait for fence for triangle
    // accel build! : \n" + errorString(err)); vkDestroyFence(logicalDevice,
    // fence, nullptr);

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // update last used build modes
    this->buildMode = mode;
    this->minimizeMemory = minimizeMemory;
    this->allowCompaction = allowCompaction;

    // note that the current tree is not yet compact
    isCompact = false;

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void update(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes) {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }

    VkResult err;

    // if we previously minimized memory, we need to reallocate our scratch buffer...
    if (minimizeMemory) {
      // Get size info
      VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
      accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
      accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
        LOG_ERROR("build mode is uninitialized!");
      } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else {
        LOG_ERROR("build mode unsupported!");
      }

      if (minimizeMemory) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
      }
      if (allowCompaction) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
      }
      accelerationStructureBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
      accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

      VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
      accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
      gprt::vkGetAccelerationStructureBuildSizes(logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                                 &accelerationStructureBuildGeometryInfo, maxPrimitiveCounts.data(),
                                                 &accelerationStructureBuildSizesInfo);

      if (scratchBuffer && scratchBuffer->size < accelerationStructureBuildSizesInfo.buildScratchSize) {
        scratchBuffer->destroy();
        delete (scratchBuffer);
        scratchBuffer = nullptr;
      }

      if (!scratchBuffer) {
        scratchBuffer =
            new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                       // means that the buffer can be used in a VkDescriptorBufferInfo. //
                       // Is this required? If not, remove this...
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                           // means we can get this buffer's address with
                           // vkGetBufferDeviceAddress
                           VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                           // means we can use this buffer as a storage buffer
                           VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                       // means that this memory is stored directly on the device
                       //  (rather than the host, or in a special host/device section)
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.buildScratchSize,
                       accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
      }
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else {
      LOG_ERROR("build mode unsupported!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
    accelerationBuildGeometryInfo.srcAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.dstAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->deviceAddress;

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(commandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for triangle accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for triangle accel build! : \n" + errorString(err));

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void compact(VkQueryPool compactedSizeQueryPool) {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (!allowCompaction) {
      LOG_ERROR("Tree must have previously been built with compaction allowed.");
    }
    if (isCompact)
      return;   // tree is already compact.

    VkResult err;

    VkDeviceSize compactedSize;

    // get size for compacted structure.
    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for triangle accel query compaction size! : \n" + errorString(err));

      // reset the query so we can use it again
      vkCmdResetQueryPool(commandBuffer, compactedSizeQueryPool, 0, 1);

      gprt::vkCmdWriteAccelerationStructuresProperties(commandBuffer, 1, &accelerationStructure,
                                                       VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR,
                                                       compactedSizeQueryPool, 0);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for triangle accel query compaction size! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for triangle accel query compaction size! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for triangle accel query compaction size! : \n" + errorString(err));

      uint64_t buffer[1] = {0};
      err = vkGetQueryPoolResults(logicalDevice, compactedSizeQueryPool, 0, 1, sizeof(VkDeviceSize), buffer,
                                  sizeof(VkDeviceSize), VK_QUERY_RESULT_WAIT_BIT);
      compactedSize = buffer[0];

      if (err)
        LOG_ERROR("failed to get query pool results for triangle accel query compaction size! : \n" + errorString(err));
    }

    // allocate compact buffer and compact acceleration structure
    if (compactBuffer && compactBuffer->size != compactedSize) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    if (!compactBuffer) {
      compactBuffer = new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                                 // means we can use this buffer as a means of storing an acceleration
                                 // structure
                                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                     // means we can get this buffer's address with
                                     // vkGetBufferDeviceAddress
                                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                     // means we can use this buffer as a storage buffer
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                 // means that this memory is stored directly on the device
                                 //  (rather than the host, or in a special host/device section)
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, compactedSize,
                                 accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = compactBuffer->buffer;
      accelerationStructureCreateInfo.size = compactedSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(logicalDevice, &accelerationStructureCreateInfo, nullptr,
                                                &compactAccelerationStructure);
      if (err)
        LOG_ERROR("failed to create compact acceleration structure for triangle accel "
                  "build! : \n" +
                  errorString(err));
    }

    // Copy over the compacted acceleration structure
    VkCopyAccelerationStructureInfoKHR copyAccelerationStructureInfo{};
    copyAccelerationStructureInfo.sType = VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR;
    copyAccelerationStructureInfo.src = accelerationStructure;
    copyAccelerationStructureInfo.dst = compactAccelerationStructure;
    copyAccelerationStructureInfo.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
    copyAccelerationStructureInfo.pNext = nullptr;

    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for triangle accel compaction! : \n" + errorString(err));

      gprt::vkCmdCopyAccelerationStructure(commandBuffer, &copyAccelerationStructureInfo);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for triangle accel compaction! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for triangle accel compaction! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for triangle accel compaction! : \n" + errorString(err));
    }

    // free the original tree and buffer
    {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    // get compact address
    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = compactAccelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // mark that the tree is now compact
    isCompact = true;
  }

  void destroy() {
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (compactAccelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
    }

    if (compactBuffer) {
      compactBuffer->destroy();
      delete compactBuffer;
      compactBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }
  };
};

struct AABBAccel : public Accel {
  std::vector<AABBGeom *> geometries;

  // Caching these for fast tree updates
  std::vector<VkAccelerationStructureBuildRangeInfoKHR> accelerationBuildStructureRangeInfos;
  std::vector<VkAccelerationStructureBuildRangeInfoKHR *> accelerationBuildStructureRangeInfoPtrs;
  std::vector<VkAccelerationStructureGeometryKHR> accelerationStructureGeometries;
  std::vector<uint32_t> maxPrimitiveCounts;

  AABBAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VmaAllocator allocator,
            VkCommandBuffer commandBuffer, VkQueue queue, size_t numGeometries, AABBGeom *geometries)
      : Accel(physicalDevice, logicalDevice, allocator, commandBuffer, queue) {
    this->geometries.resize(numGeometries);
    memcpy(this->geometries.data(), geometries, sizeof(GPRTGeom *) * numGeometries);
  };

  ~AABBAccel(){};

  AccelType getType() { return GPRT_AABB_ACCEL; }

  size_t getSize() {
    size_t size = 0;
    if (accelBuffer)
      size += accelBuffer->getSize();
    if (compactBuffer)
      size += compactBuffer->getSize();
    if (scratchBuffer)
      size += scratchBuffer->getSize();
    return size;
  };

  void build(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes,
             GPRTBuildMode mode, bool allowCompaction, bool minimizeMemory) {
    VkResult err;

    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());
    maxPrimitiveCounts.resize(geometries.size());

    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
      //   means, anyhit shader is disabled

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...

      geom.geometryType = VK_GEOMETRY_TYPE_AABBS_KHR;

      // aabb data
      geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
      geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
      geom.geometry.aabbs.data.deviceAddress = geometries[gid]->aabb.buffers[0]->deviceAddress;
      geom.geometry.aabbs.stride = geometries[gid]->aabb.stride;

      auto &geomRange = accelerationBuildStructureRangeInfos[gid];
      accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
      geomRange.primitiveCount = geometries[gid]->aabb.count;
      geomRange.primitiveOffset = geometries[gid]->aabb.offset;
      geomRange.firstVertex = 0;   // unused
      geomRange.transformOffset = 0;

      maxPrimitiveCounts[gid] = geometries[gid]->aabb.count;
    }

    // Get size info
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (mode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (mode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationStructureBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                               &accelerationStructureBuildGeometryInfo, maxPrimitiveCounts.data(),
                                               &accelerationStructureBuildSizesInfo);

    // If previously compacted, free those resources up.
    if (compactBuffer) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    if (accelBuffer && accelBuffer->size < accelerationStructureBuildSizesInfo.accelerationStructureSize) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    if (!accelBuffer) {
      accelBuffer =
          new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                     // means we can use this buffer as a means of storing an acceleration
                     // structure
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                         // means we can get this buffer's address with
                         // vkGetBufferDeviceAddress
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         // means we can use this buffer as a storage buffer
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     // means that this memory is stored directly on the device
                     //  (rather than the host, or in a special host/device section)
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.accelerationStructureSize,
                     accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(logicalDevice, &accelerationStructureCreateInfo, nullptr,
                                                &accelerationStructure);
      if (err)
        LOG_ERROR("failed to create acceleration structure for AABB accel "
                  "build! : \n" +
                  errorString(err));
    }

    if (scratchBuffer && scratchBuffer->size < accelerationStructureBuildSizesInfo.buildScratchSize) {
      scratchBuffer->destroy();
      delete (scratchBuffer);
      scratchBuffer = nullptr;
    }

    if (!scratchBuffer) {
      scratchBuffer =
          new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                     // means that the buffer can be used in a VkDescriptorBufferInfo. //
                     // Is this required? If not, remove this...
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                         // means we can get this buffer's address with
                         // vkGetBufferDeviceAddress
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         // means we can use this buffer as a storage buffer
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     // means that this memory is stored directly on the device
                     //  (rather than the host, or in a special host/device section)
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.buildScratchSize,
                     accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (mode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (mode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->deviceAddress;

    // Build the acceleration structure on the device via a one-time command
    // buffer submission Some implementations may support acceleration structure
    // building on the host
    // (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands),
    // but we prefer device builds VkCommandBuffer commandBuffer =
    // vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(commandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for AABB accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for AABB accel build! : \n" + errorString(err));

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // update last used build modes
    this->buildMode = mode;
    this->minimizeMemory = minimizeMemory;
    this->allowCompaction = allowCompaction;

    // note that the current tree is not yet compact
    isCompact = false;

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void update(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes) {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }

    VkResult err;

    // if we previously minimized memory, we need to reallocate our scratch buffer...
    if (minimizeMemory) {
      // Get size info
      VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
      accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
      accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
        LOG_ERROR("build mode is uninitialized!");
      } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else {
        LOG_ERROR("build mode not recognized!");
      }

      if (minimizeMemory) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
      }
      if (allowCompaction) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
      }
      accelerationStructureBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
      accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

      VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
      accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
      gprt::vkGetAccelerationStructureBuildSizes(logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                                 &accelerationStructureBuildGeometryInfo, maxPrimitiveCounts.data(),
                                                 &accelerationStructureBuildSizesInfo);

      if (scratchBuffer && scratchBuffer->size < accelerationStructureBuildSizesInfo.buildScratchSize) {
        scratchBuffer->destroy();
        delete (scratchBuffer);
        scratchBuffer = nullptr;
      }

      if (!scratchBuffer) {
        scratchBuffer =
            new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                       // means that the buffer can be used in a VkDescriptorBufferInfo. //
                       // Is this required? If not, remove this...
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                           // means we can get this buffer's address with
                           // vkGetBufferDeviceAddress
                           VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                           // means we can use this buffer as a storage buffer
                           VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                       // means that this memory is stored directly on the device
                       //  (rather than the host, or in a special host/device section)
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.buildScratchSize,
                       accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
      }
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else {
      LOG_ERROR("build mode unsupported!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
    accelerationBuildGeometryInfo.srcAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.dstAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->deviceAddress;

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(commandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for AABB accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for AABB accel build! : \n" + errorString(err));

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void compact(VkQueryPool compactedSizeQueryPool) {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (!allowCompaction) {
      LOG_ERROR("Tree must have previously been built with compaction allowed.");
    }
    if (isCompact)
      return;   // tree is already compact.

    VkResult err;

    VkDeviceSize compactedSize;

    // get size for compacted structure.
    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for aabb accel query compaction size! : \n" + errorString(err));

      // reset the query so we can use it again
      vkCmdResetQueryPool(commandBuffer, compactedSizeQueryPool, 0, 1);

      gprt::vkCmdWriteAccelerationStructuresProperties(commandBuffer, 1, &accelerationStructure,
                                                       VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR,
                                                       compactedSizeQueryPool, 0);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for aabb accel query compaction size! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for aabb accel query compaction size! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for aabb accel query compaction size! : \n" + errorString(err));

      uint64_t buffer[1] = {0};
      err = vkGetQueryPoolResults(logicalDevice, compactedSizeQueryPool, 0, 1, sizeof(VkDeviceSize), buffer,
                                  sizeof(VkDeviceSize), VK_QUERY_RESULT_WAIT_BIT);
      compactedSize = buffer[0];

      if (err)
        LOG_ERROR("failed to get query pool results for aabb accel query compaction size! : \n" + errorString(err));
    }

    // allocate compact buffer and compact acceleration structure
    if (compactBuffer && compactBuffer->size != compactedSize) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    if (!compactBuffer) {
      compactBuffer = new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                                 // means we can use this buffer as a means of storing an acceleration
                                 // structure
                                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                     // means we can get this buffer's address with
                                     // vkGetBufferDeviceAddress
                                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                     // means we can use this buffer as a storage buffer
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                 // means that this memory is stored directly on the device
                                 //  (rather than the host, or in a special host/device section)
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, compactedSize,
                                 accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = compactBuffer->buffer;
      accelerationStructureCreateInfo.size = compactedSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(logicalDevice, &accelerationStructureCreateInfo, nullptr,
                                                &compactAccelerationStructure);
      if (err)
        LOG_ERROR("failed to create compact acceleration structure for aabb accel "
                  "build! : \n" +
                  errorString(err));
    }

    // Copy over the compacted acceleration structure
    VkCopyAccelerationStructureInfoKHR copyAccelerationStructureInfo{};
    copyAccelerationStructureInfo.sType = VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR;
    copyAccelerationStructureInfo.src = accelerationStructure;
    copyAccelerationStructureInfo.dst = compactAccelerationStructure;
    copyAccelerationStructureInfo.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
    copyAccelerationStructureInfo.pNext = nullptr;

    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for aabb accel compaction! : \n" + errorString(err));

      gprt::vkCmdCopyAccelerationStructure(commandBuffer, &copyAccelerationStructureInfo);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for aabb accel compaction! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for aabb accel compaction! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for aabb accel compaction! : \n" + errorString(err));
    }

    // free the original tree and buffer
    {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    // get compact address
    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = compactAccelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // mark that the tree is now compact
    isCompact = true;
  }

  void destroy() {
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (compactAccelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
    }

    if (compactBuffer) {
      compactBuffer->destroy();
      delete compactBuffer;
      compactBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }
  };
};

struct InstanceAccel : public Accel {
  uint32_t numInstances;
  std::vector<Accel *> instances;

  // the total number of geometries referenced by this instance accel's BLASes
  uint32_t numGeometries = -1;

  // caching these for fast updates
  size_t instanceOffset = -1;
  uint64_t referencesAddress;
  uint64_t visibilityMasksAddress;
  uint64_t instanceOffsetsAddress;
  uint64_t transformBufferAddress;

  Buffer *instancesBuffer = nullptr;
  Buffer *accelAddressesBuffer = nullptr;
  Buffer *instanceOffsetsBuffer = nullptr;

  struct {
    Buffer *buffer = nullptr;
    uint32_t stride = 0;
    uint32_t offset = 0;
  } transforms;

  struct {
    Buffer *buffer = nullptr;
    // uint32_t stride = 0;
    // uint32_t offset = 0;
  } references;

  struct {
    Buffer *buffer = nullptr;
    // uint32_t stride = 0;
    // uint32_t offset = 0;
  } visibilityMasks;

  struct {
    Buffer *buffer = nullptr;
    // uint32_t stride = 0;
    // uint32_t offset = 0;
  } offsets;

  // todo, accept this in constructor
  VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

  InstanceAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VmaAllocator allocator,
                VkCommandBuffer commandBuffer, VkQueue queue, uint32_t numInstances, GPRTAccel *instances)
      : Accel(physicalDevice, logicalDevice, allocator, commandBuffer, queue) {
    this->numInstances = numInstances;

    if (instances) {
      this->instances.resize(numInstances);
      memcpy(this->instances.data(), instances, sizeof(GPRTAccel *) * numInstances);

      // count number of geometry referenced.
      uint32_t numGeometry = 0;
      for (uint32_t j = 0; j < this->instances.size(); ++j) {
        if (this->instances[j]->getType() == GPRT_TRIANGLE_ACCEL) {
          TriangleAccel *triangleAccel = (TriangleAccel *) this->instances[j];
          numGeometry += (uint32_t) triangleAccel->geometries.size();
        } else if (this->instances[j]->getType() == GPRT_AABB_ACCEL) {
          AABBAccel *aabbAccel = (AABBAccel *) this->instances[j];
          numGeometry += (uint32_t) aabbAccel->geometries.size();
        } else {
          LOG_ERROR("Unaccounted for BLAS type!");
        }
      }
      this->numGeometries = numGeometry;
    }

    instancesBuffer = new Buffer(physicalDevice, logicalDevice, allocator, commandBuffer, queue,
                                 // I guess I need this to use these buffers as input to tree builds?
                                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                                     // means we can get this buffer's address with
                                     // vkGetBufferDeviceAddress
                                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                     // means we can use this buffer as a storage buffer
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                 // means that this memory is stored directly on the device
                                 //  (rather than the host, or in a special host/device section)
                                 // VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                 // VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, // temporary
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                 sizeof(VkAccelerationStructureInstanceKHR) * numInstances, 16);
  };

  ~InstanceAccel(){};

  void setTransforms(Buffer *transforms, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 transform
    // per instance
    this->transforms.buffer = transforms;
    this->transforms.stride = stride;
    this->transforms.offset = offset;
  }

  void setReferences(Buffer *references = nullptr   //,
                                                    // size_t count,
                                                    // size_t stride,
                                                    // size_t offset
  ) {
    this->references.buffer = references;
  }

  void setVisibilityMasks(Buffer *masks = nullptr   //,
                                                    // size_t count,
                                                    // size_t stride,
                                                    // size_t offset
  ) {
    this->visibilityMasks.buffer = masks;
  }

  void setOffsets(Buffer *offsets = nullptr   //,
                                              // size_t count,
                                              // size_t stride,
                                              // size_t offset
  ) {
    this->offsets.buffer = offsets;
  }

  void setNumGeometries(uint32_t numGeometries) { this->numGeometries = numGeometries; }

  uint32_t getNumGeometries() {
    if (this->numGeometries == -1) {
      LOG_ERROR("Error, numGeometries for this instance must be set by the user!");
    }
    return this->numGeometries;
  }

  AccelType getType() { return GPRT_INSTANCE_ACCEL; }

  size_t getSize() { throw std::runtime_error("Error, not implemeted!"); };

  void build(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes,
             GPRTBuildMode mode, bool allowCompaction, bool minimizeMemory) {
    VkResult err;

    // Compute the instance offset for the SBT record.
    //   The instance shader binding table record offset is the total number
    //   of geometries referenced by all instances up until this instance tree
    //   multiplied by the number of ray types.
    instanceOffset = 0;
    for (uint32_t i = 0; i < accels.size(); ++i) {
      if (accels[i] == this)
        break;
      if (accels[i]->getType() == GPRT_INSTANCE_ACCEL) {
        InstanceAccel *instanceAccel = (InstanceAccel *) accels[i];
        size_t numGeometry = instanceAccel->getNumGeometries();
        instanceOffset += numGeometry * numRayTypes;
      }
    }

    // No instance addressed provided, so we need to supply our own.
    if (references.buffer == nullptr) {
      // delete if not big enough
      if (accelAddressesBuffer && accelAddressesBuffer->size != numInstances * sizeof(uint64_t)) {
        accelAddressesBuffer->destroy();
        delete accelAddressesBuffer;
        accelAddressesBuffer = nullptr;
      }

      // make buffer if not made yet
      if (accelAddressesBuffer == nullptr) {
        accelAddressesBuffer = new Buffer(physicalDevice, logicalDevice, allocator, commandBuffer, queue,
                                          // I guess I need this to use these buffers as input to tree builds?
                                          VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                                              // means we can get this buffer's address with
                                              // vkGetBufferDeviceAddress
                                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                              // means we can use this buffer as a storage buffer
                                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                          // means that this memory is stored directly on the device
                                          //  (rather than the host, or in a special host/device section)
                                          // VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | // temporary (doesn't work
                                          // on AMD)
                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,   // temporary
                                          sizeof(uint64_t) * numInstances, 16);
      }

      // transfer over addresses
      std::vector<uint64_t> addresses(numInstances);
      for (uint32_t i = 0; i < numInstances; ++i)
        addresses[i] = this->instances[i]->address;
      accelAddressesBuffer->map();
      memcpy(accelAddressesBuffer->mapped, addresses.data(), sizeof(uint64_t) * numInstances);
      accelAddressesBuffer->unmap();
      referencesAddress = accelAddressesBuffer->deviceAddress;
    }
    // Instance acceleration structure references provided by the user
    else {
      referencesAddress = references.buffer->deviceAddress;
    }

    // If the visibility mask address is -1, we assume a mask of 0xFF
    visibilityMasksAddress = -1;
    if (visibilityMasks.buffer != nullptr) {
      visibilityMasksAddress = visibilityMasks.buffer->deviceAddress;
    }

    // No instance offsets provided, so we need to supply our own.
    if (offsets.buffer == nullptr) {
      // delete if not big enough
      if (instanceOffsetsBuffer && instanceOffsetsBuffer->size != numInstances * sizeof(uint64_t)) {
        instanceOffsetsBuffer->destroy();
        delete instanceOffsetsBuffer;
        instanceOffsetsBuffer = nullptr;
      }

      // make buffer if not made yet
      if (instanceOffsetsBuffer == nullptr) {
        instanceOffsetsBuffer = new Buffer(physicalDevice, logicalDevice, allocator, commandBuffer, queue,
                                           // I guess I need this to use these buffers as input to tree builds?
                                           VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                                               // means we can get this buffer's address with
                                               // vkGetBufferDeviceAddress
                                               VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                               // means we can use this buffer as a storage buffer
                                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                           // means that this memory is stored directly on the device
                                           // (rather than the host, or in a special host/device section)
                                           // VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | // temporary (doesn't work
                                           // on AMD)
                                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,   // temporary
                                           sizeof(uint64_t) * numInstances, 16);
      }

      // transfer over offsets
      std::vector<int32_t> blasOffsets(numInstances);
      int offset = 0;
      for (uint32_t i = 0; i < numInstances; ++i) {
        blasOffsets[i] = offset;

        if (this->instances[i]->getType() == GPRT_TRIANGLE_ACCEL) {
          TriangleAccel *triAccel = (TriangleAccel *) this->instances[i];
          offset += (uint32_t) triAccel->geometries.size() * numRayTypes;
        } else if (this->instances[i]->getType() == GPRT_AABB_ACCEL) {
          AABBAccel *aabbAccel = (AABBAccel *) this->instances[i];
          offset += (uint32_t) aabbAccel->geometries.size() * numRayTypes;
        } else {
          LOG_ERROR("Error, unknown instance type");
        }
      }
      instanceOffsetsBuffer->map();
      memcpy(instanceOffsetsBuffer->mapped, blasOffsets.data(), sizeof(int32_t) * numInstances);
      instanceOffsetsBuffer->unmap();
      instanceOffsetsAddress = instanceOffsetsBuffer->deviceAddress;
    }
    // Instance acceleration structure references provided by the user
    else {
      instanceOffsetsAddress = offsets.buffer->deviceAddress;
    }

    // If transform buffer address is -1, we assume an identity transformation.
    transformBufferAddress = -1;
    if (transforms.buffer != nullptr) {
      transformBufferAddress = transforms.buffer->deviceAddress;
    }

    // Use a compute shader to copy transforms into instances buffer
    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      struct PushConstants {
        uint64_t instanceBufferAddr;
        uint64_t transformBufferAddr;
        uint64_t accelReferencesAddr;
        uint64_t instanceShaderBindingTableRecordOffset;
        uint64_t transformOffset;
        uint64_t transformStride;
        uint64_t instanceOffsetsBufferAddr;
        uint64_t instanceVisibilityMasksBufferAddr;
        uint64_t pad[16 - 8];
      } pushConstants;

      pushConstants.instanceBufferAddr = instancesBuffer->deviceAddress;
      pushConstants.transformBufferAddr = transformBufferAddress;
      pushConstants.accelReferencesAddr = referencesAddress;
      pushConstants.instanceShaderBindingTableRecordOffset = instanceOffset;
      pushConstants.transformOffset = transforms.offset;
      pushConstants.transformStride = transforms.stride;
      pushConstants.instanceOffsetsBufferAddr = instanceOffsetsAddress;
      pushConstants.instanceVisibilityMasksBufferAddr = visibilityMasksAddress;

      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      vkCmdPushConstants(commandBuffer, internalStages["gprtFillInstanceData"].layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                         sizeof(PushConstants), &pushConstants);
      vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, internalStages["gprtFillInstanceData"].pipeline);
      // vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
      // pipelineLayout, 0, 1, &descriptorSet, 0, 0);
      vkCmdDispatch(commandBuffer, numInstances, 1, 1);
      err = vkEndCommandBuffer(commandBuffer);

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for instance accel build! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for instance accel build! : \n" + errorString(err));
    }

    VkAccelerationStructureGeometryKHR accelerationStructureGeometry{};
    accelerationStructureGeometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    accelerationStructureGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    accelerationStructureGeometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    accelerationStructureGeometry.geometry.instances.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    accelerationStructureGeometry.geometry.instances.arrayOfPointers = VK_FALSE;
    accelerationStructureGeometry.geometry.instances.data.deviceAddress = instancesBuffer->deviceAddress;

    // Get size info
    /*
    The pSrcAccelerationStructure, dstAccelerationStructure, and mode members of
    pBuildInfo are ignored. Any VkDeviceOrHostAddressKHR members of pBuildInfo
    are ignored by this command, except that the hostAddress member of
    VkAccelerationStructureGeometryTrianglesDataKHR::transformData will be
    examined to check if it is NULL.*
    */
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    if (mode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (mode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationStructureBuildGeometryInfo.geometryCount = 1;
    accelerationStructureBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;

    uint32_t primitive_count = numInstances;

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                               &accelerationStructureBuildGeometryInfo, &primitive_count,
                                               &accelerationStructureBuildSizesInfo);

    // If previously compacted, free those resources up.
    if (compactBuffer) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    if (accelBuffer && accelBuffer->size != accelerationStructureBuildSizesInfo.accelerationStructureSize) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    if (!accelBuffer) {
      accelBuffer =
          new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                     // means we can use this buffer as a means of storing an acceleration
                     // structure
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                         // means we can get this buffer's address with
                         // vkGetBufferDeviceAddress
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         // means we can use this buffer as a storage buffer
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     // means that this memory is stored directly on the device
                     //  (rather than the host, or in a special host/device section)
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.accelerationStructureSize,
                     accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(logicalDevice, &accelerationStructureCreateInfo, nullptr,
                                                &accelerationStructure);
      if (err)
        LOG_ERROR("failed to create acceleration structure for instance accel "
                  "build! : \n" +
                  errorString(err));
    }

    if (scratchBuffer && scratchBuffer->size != accelerationStructureBuildSizesInfo.buildScratchSize) {
      scratchBuffer->destroy();
      delete (scratchBuffer);
      scratchBuffer = nullptr;
    }

    if (!scratchBuffer) {
      scratchBuffer =
          new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                     // means that the buffer can be used in a VkDescriptorBufferInfo. //
                     // Is this required? If not, remove this...
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                         // means we can get this buffer's address with
                         // vkGetBufferDeviceAddress
                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         // means we can use this buffer as a storage buffer
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     // means that this memory is stored directly on the device
                     //  (rather than the host, or in a special host/device section)
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.buildScratchSize,
                     accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    if (mode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (mode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (mode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = 1;
    accelerationBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->deviceAddress;

    VkAccelerationStructureBuildRangeInfoKHR accelerationStructureBuildRangeInfo{};
    accelerationStructureBuildRangeInfo.primitiveCount = numInstances;
    accelerationStructureBuildRangeInfo.primitiveOffset = 0;
    accelerationStructureBuildRangeInfo.firstVertex = 0;
    accelerationStructureBuildRangeInfo.transformOffset = 0;
    std::vector<VkAccelerationStructureBuildRangeInfoKHR *> accelerationBuildStructureRangeInfoPtrs = {
        &accelerationStructureBuildRangeInfo};

    // // Build the acceleration structure on the device via a one-time command
    // buffer submission
    // // Some implementations may support acceleration structure building on
    // the host
    // (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands),
    // but we prefer device builds

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for instance accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(commandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for instance accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueWaitIdle(queue);

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for instance accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for instance accel build! : \n" + errorString(err));

    // // // Wait for the fence to signal that command buffer has finished
    // executing
    // // err = vkWaitForFences(logicalDevice, 1, &fence, VK_TRUE, 100000000000
    // /*timeout*/);
    // // if (err) LOG_ERROR("failed to wait for fence for instance accel
    // build! : \n" + errorString(err));
    // // vkDestroyFence(logicalDevice, fence, nullptr);

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // update last used build modes
    this->buildMode = mode;
    this->minimizeMemory = minimizeMemory;
    this->allowCompaction = allowCompaction;

    // note that the current tree is not yet compact
    isCompact = false;

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void update(std::map<std::string, Stage> internalStages, std::vector<Accel *> accels, uint32_t numRayTypes) {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }

    VkResult err;

    // assuming that instanceOffset for the SBT record has not changed...
    // assuming that referencesAddress is already configured from previous build
    // assuming that visibilityMasksAddress is already configured from previous build
    // assuming instanceOffsetsAddress is already configured from previous build
    // assuming transformBufferAddress is already configured from previous build

    // Use a compute shader to copy data into instances buffer
    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      struct PushConstants {
        uint64_t instanceBufferAddr;
        uint64_t transformBufferAddr;
        uint64_t accelReferencesAddr;
        uint64_t instanceShaderBindingTableRecordOffset;
        uint64_t transformOffset;
        uint64_t transformStride;
        uint64_t instanceOffsetsBufferAddr;
        uint64_t instanceVisibilityMasksBufferAddr;
        uint64_t pad[16 - 8];
      } pushConstants;

      pushConstants.instanceBufferAddr = instancesBuffer->deviceAddress;
      pushConstants.transformBufferAddr = transformBufferAddress;
      pushConstants.accelReferencesAddr = referencesAddress;
      pushConstants.instanceShaderBindingTableRecordOffset = instanceOffset;
      pushConstants.transformOffset = transforms.offset;
      pushConstants.transformStride = transforms.stride;
      pushConstants.instanceOffsetsBufferAddr = instanceOffsetsAddress;
      pushConstants.instanceVisibilityMasksBufferAddr = visibilityMasksAddress;

      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      vkCmdPushConstants(commandBuffer, internalStages["gprtFillInstanceData"].layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                         sizeof(PushConstants), &pushConstants);
      vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, internalStages["gprtFillInstanceData"].pipeline);
      // vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
      // pipelineLayout, 0, 1, &descriptorSet, 0, 0);
      vkCmdDispatch(commandBuffer, numInstances, 1, 1);
      err = vkEndCommandBuffer(commandBuffer);

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for instance accel build! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for instance accel build! : \n" + errorString(err));
    }

    VkAccelerationStructureGeometryKHR accelerationStructureGeometry{};
    accelerationStructureGeometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    accelerationStructureGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    accelerationStructureGeometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    accelerationStructureGeometry.geometry.instances.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    accelerationStructureGeometry.geometry.instances.arrayOfPointers = VK_FALSE;
    accelerationStructureGeometry.geometry.instances.data.deviceAddress = instancesBuffer->deviceAddress;

    // if we previously minimized memory, we need to reallocate our scratch buffer...
    if (minimizeMemory) {
      // Get size info
      VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
      accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
      accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
      if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
        LOG_ERROR("build mode is uninitialized!");
      } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else {
        LOG_ERROR("build mode unsupported!");
      }

      if (minimizeMemory) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
      }
      if (allowCompaction) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
      }
      accelerationStructureBuildGeometryInfo.geometryCount = 1;
      accelerationStructureBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;

      uint32_t primitive_count = numInstances;

      VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
      accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
      gprt::vkGetAccelerationStructureBuildSizes(logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                                 &accelerationStructureBuildGeometryInfo, &primitive_count,
                                                 &accelerationStructureBuildSizesInfo);

      if (scratchBuffer && scratchBuffer->size != accelerationStructureBuildSizesInfo.buildScratchSize) {
        scratchBuffer->destroy();
        delete (scratchBuffer);
        scratchBuffer = nullptr;
      }

      if (!scratchBuffer) {
        scratchBuffer =
            new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                       // means that the buffer can be used in a VkDescriptorBufferInfo. //
                       // Is this required? If not, remove this...
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                           // means we can get this buffer's address with
                           // vkGetBufferDeviceAddress
                           VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                           // means we can use this buffer as a storage buffer
                           VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                       // means that this memory is stored directly on the device
                       //  (rather than the host, or in a special host/device section)
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.buildScratchSize,
                       accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
      }
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else {
      LOG_ERROR("build mode unsupported!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
    accelerationBuildGeometryInfo.srcAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.dstAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = 1;
    accelerationBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->deviceAddress;

    VkAccelerationStructureBuildRangeInfoKHR accelerationStructureBuildRangeInfo{};
    accelerationStructureBuildRangeInfo.primitiveCount = numInstances;
    accelerationStructureBuildRangeInfo.primitiveOffset = 0;
    accelerationStructureBuildRangeInfo.firstVertex = 0;
    accelerationStructureBuildRangeInfo.transformOffset = 0;
    std::vector<VkAccelerationStructureBuildRangeInfoKHR *> accelerationBuildStructureRangeInfoPtrs = {
        &accelerationStructureBuildRangeInfo};

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for instance accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(commandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for instance accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueWaitIdle(queue);

    err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for instance accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for instance accel build! : \n" + errorString(err));

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void compact(VkQueryPool compactedSizeQueryPool) {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (!allowCompaction) {
      LOG_ERROR("Tree must have previously been built with compaction allowed.");
    }
    if (isCompact)
      return;   // tree is already compact.

    VkResult err;

    VkDeviceSize compactedSize;

    // get size for compacted structure.
    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for instance accel query compaction size! : \n" + errorString(err));

      // reset the query so we can use it again
      vkCmdResetQueryPool(commandBuffer, compactedSizeQueryPool, 0, 1);

      gprt::vkCmdWriteAccelerationStructuresProperties(commandBuffer, 1, &accelerationStructure,
                                                       VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR,
                                                       compactedSizeQueryPool, 0);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for instance accel query compaction size! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for instance accel query compaction size! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for instance accel query compaction size! : \n" + errorString(err));

      uint64_t buffer[1] = {0};
      err = vkGetQueryPoolResults(logicalDevice, compactedSizeQueryPool, 0, 1, sizeof(VkDeviceSize), buffer,
                                  sizeof(VkDeviceSize), VK_QUERY_RESULT_WAIT_BIT);
      compactedSize = buffer[0];

      if (err)
        LOG_ERROR("failed to get query pool results for instance accel query compaction size! : \n" + errorString(err));
    }

    // allocate compact buffer and compact acceleration structure
    if (compactBuffer && compactBuffer->size != compactedSize) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    if (!compactBuffer) {
      compactBuffer = new Buffer(physicalDevice, logicalDevice, allocator, VK_NULL_HANDLE, VK_NULL_HANDLE,
                                 // means we can use this buffer as a means of storing an acceleration
                                 // structure
                                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                     // means we can get this buffer's address with
                                     // vkGetBufferDeviceAddress
                                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                     // means we can use this buffer as a storage buffer
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                 // means that this memory is stored directly on the device
                                 //  (rather than the host, or in a special host/device section)
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, compactedSize,
                                 accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = compactBuffer->buffer;
      accelerationStructureCreateInfo.size = compactedSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(logicalDevice, &accelerationStructureCreateInfo, nullptr,
                                                &compactAccelerationStructure);
      if (err)
        LOG_ERROR("failed to create compact acceleration structure for instance accel "
                  "build! : \n" +
                  errorString(err));
    }

    // Copy over the compacted acceleration structure
    VkCopyAccelerationStructureInfoKHR copyAccelerationStructureInfo{};
    copyAccelerationStructureInfo.sType = VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR;
    copyAccelerationStructureInfo.src = accelerationStructure;
    copyAccelerationStructureInfo.dst = compactAccelerationStructure;
    copyAccelerationStructureInfo.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
    copyAccelerationStructureInfo.pNext = nullptr;

    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for instance accel compaction! : \n" + errorString(err));

      gprt::vkCmdCopyAccelerationStructure(commandBuffer, &copyAccelerationStructureInfo);

      err = vkEndCommandBuffer(commandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for instance accel compaction! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for instance accel compaction! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for instance accel compaction! : \n" + errorString(err));
    }

    // free the original tree and buffer
    {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    // get compact address
    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = compactAccelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);

    // mark that the tree is now compact
    isCompact = true;
  }

  void destroy() {
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (compactAccelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
    }

    if (compactBuffer) {
      compactBuffer->destroy();
      delete compactBuffer;
      compactBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }

    if (instancesBuffer) {
      instancesBuffer->destroy();
      delete instancesBuffer;
      instancesBuffer = nullptr;
    }

    if (accelAddressesBuffer) {
      accelAddressesBuffer->destroy();
      delete accelAddressesBuffer;
      accelAddressesBuffer = nullptr;
    }

    if (instanceOffsetsBuffer) {
      instanceOffsetsBuffer->destroy();
      delete instanceOffsetsBuffer;
      instanceOffsetsBuffer = nullptr;
    }
  };
};

struct Context {
  VkApplicationInfo appInfo;

  // Vulkan instance, stores all per-application states
  VkInstance instance;
  std::vector<std::string> supportedInstanceExtensions;

  // optional windowing features
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  GLFWwindow *window = nullptr;
  VkExtent2D windowExtent;
  VkPresentModeKHR presentMode;
  VkSurfaceFormatKHR surfaceFormat;
  VkSurfaceCapabilitiesKHR surfaceCapabilities;
  uint32_t surfaceImageCount;
  VkSwapchainKHR swapchain = VK_NULL_HANDLE;
  std::vector<VkImage> swapchainImages;
  uint32_t currentImageIndex;
  VkSemaphore imageAvailableSemaphore = VK_NULL_HANDLE;
  VkSemaphore renderFinishedSemaphore = VK_NULL_HANDLE;
  VkFence inFlightFence = VK_NULL_HANDLE;

  struct ImGuiData {
    uint32_t width = -1;
    uint32_t height = -1;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkFramebuffer frameBuffer = VK_NULL_HANDLE;
    Texture *colorAttachment = nullptr;
    Texture *depthAttachment = nullptr;
  } imgui;

  // Physical device (GPU) that Vulkan will use
  VkPhysicalDevice physicalDevice;
  // Stores physical device properties (for e.g. checking device limits)
  VkPhysicalDeviceProperties deviceProperties;
  VkPhysicalDeviceRayTracingPipelinePropertiesKHR rayTracingPipelineProperties;
  VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures;
  // Stores the features available on the selected physical device (for e.g.
  // checking if a feature is available)
  VkPhysicalDeviceFeatures deviceFeatures;
  // Stores all available memory (type) properties for the physical device
  VkPhysicalDeviceMemoryProperties deviceMemoryProperties;

  // For handling vulkan memory allocation
  VmaAllocator allocator;

  /** @brief Queue family properties of the chosen physical device */
  std::vector<VkQueueFamilyProperties> queueFamilyProperties;

  /** @brief Contains queue family indices */
  struct {
    uint32_t graphics;
    uint32_t compute;
    uint32_t transfer;
  } queueFamilyIndices;

  VkCommandBuffer graphicsCommandBuffer;
  VkCommandBuffer computeCommandBuffer;
  VkCommandBuffer transferCommandBuffer;

  /** @brief List of extensions supported by the chosen physical device */
  std::vector<std::string> supportedExtensions;

  /** @brief Set of physical device features to be enabled (must be set in the
   * derived constructor) */
  VkPhysicalDeviceFeatures enabledFeatures{};
  /** @brief Set of device extensions to be enabled for this example (must be
   * set in the derived constructor) */
  std::vector<const char *> enabledDeviceExtensions;
  std::vector<const char *> enabledInstanceExtensions;
  /** @brief Optional pNext structure for passing extension structures to device
   * creation */
  void *deviceCreatepNextChain = nullptr;
  /** @brief Logical device, application's view of the physical device (GPU) */
  VkDevice logicalDevice;

  // Handle to the device graphics queue that command buffers are submitted to
  VkQueue graphicsQueue;
  VkQueue computeQueue;
  VkQueue transferQueue;

  // Depth buffer format (selected during Vulkan initialization)
  VkFormat depthFormat;
  // Command buffer pool
  VkCommandPool graphicsCommandPool;
  VkCommandPool computeCommandPool;
  VkCommandPool transferCommandPool;
  VkQueryPool queryPool;
  VkQueryPool compactedSizeQueryPool;
  bool queryRequested = false;

  /** @brief Pipeline stages used to wait at for graphics queue submissions */
  VkPipelineStageFlags submitPipelineStages =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;   // not sure I need this one
  // Contains command buffers and semaphores to be presented to the queue
  VkSubmitInfo submitInfo;
  // Command buffers used for rendering
  // std::vector<VkCommandBuffer> drawCmdBuffers;
  // Global render pass for frame buffer writes
  VkRenderPass renderPass = VK_NULL_HANDLE;
  // List of available frame buffers (same as number of swap chain images)
  std::vector<VkFramebuffer> frameBuffers;
  // Active frame buffer index
  uint32_t currentBuffer = 0;

  // Pipeline cache object
  VkPipelineCache pipelineCache;

  // ray tracing pipeline and layout
  bool raytracingPipelineOutOfDate = true;
  VkPipeline raytracingPipeline = VK_NULL_HANDLE;
  VkPipelineLayout raytracingPipelineLayout = VK_NULL_HANDLE;

  std::vector<RayGen *> raygenPrograms;
  std::vector<Miss *> missPrograms;
  std::vector<GeomType *> geomTypes;

  bool computePipelinesOutOfDate = true;
  bool rasterPipelinesOutOfDate = true;

  std::vector<Accel *> accels;

  Sampler *defaultSampler = nullptr;
  Texture *defaultTexture1D = nullptr;
  Texture *defaultTexture2D = nullptr;
  Texture *defaultTexture3D = nullptr;
  Buffer *defaultBuffer = nullptr;

  Buffer *rasterRecordBuffer = nullptr;
  Buffer *computeRecordBuffer = nullptr;

  VkDescriptorPool samplerDescriptorPool = VK_NULL_HANDLE;
  VkDescriptorPool texture1DDescriptorPool = VK_NULL_HANDLE;
  VkDescriptorPool texture2DDescriptorPool = VK_NULL_HANDLE;
  VkDescriptorPool texture3DDescriptorPool = VK_NULL_HANDLE;
  VkDescriptorPool bufferDescriptorPool = VK_NULL_HANDLE;
  VkDescriptorPool rasterRecordDescriptorPool = VK_NULL_HANDLE;
  VkDescriptorPool computeRecordDescriptorPool = VK_NULL_HANDLE;

  VkDescriptorPool imguiPool = VK_NULL_HANDLE;

  // used to determine what descriptor sets need rebuilding
  uint32_t previousNumSamplers = 0;
  uint32_t previousNumTexture1Ds = 0;
  uint32_t previousNumTexture2Ds = 0;
  uint32_t previousNumTexture3Ds = 0;
  uint32_t previousNumBuffers = 0;
  uint32_t previousNumRasterRecords = 0;
  uint32_t previousNumComputeRecords = 0;

  VkDescriptorSetLayout samplerDescriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout texture1DDescriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout texture2DDescriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout texture3DDescriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout bufferDescriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout rasterRecordDescriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout computeRecordDescriptorSetLayout = VK_NULL_HANDLE;

  VkDescriptorSet samplerDescriptorSet = VK_NULL_HANDLE;
  VkDescriptorSet texture1DDescriptorSet = VK_NULL_HANDLE;
  VkDescriptorSet texture2DDescriptorSet = VK_NULL_HANDLE;
  VkDescriptorSet texture3DDescriptorSet = VK_NULL_HANDLE;
  VkDescriptorSet bufferDescriptorSet = VK_NULL_HANDLE;

  VkDescriptorSet rasterRecordDescriptorSet = VK_NULL_HANDLE;
  VkDescriptorSet computeRecordDescriptorSet = VK_NULL_HANDLE;

  std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups{};
  Buffer *raygenTable = nullptr;
  Buffer *missTable = nullptr;
  Buffer *hitgroupTable = nullptr;

  // uint32_t numRayTypes = 1;

  // struct InternalStages {
  //   // for copying transforms into the instance buffer
  //   std::string fillInstanceDataEntryPoint = "gprtFillInstanceData";
  //   VkPipelineLayout fillInstanceDataPipelineLayout;
  //   VkShaderModule fillInstanceDataShaderModule;
  //   VkPipeline fillInstanceDataPipeline;
  // }
  Stage fillInstanceDataStage;
  Module *internalModule = nullptr;

  struct SortStages {
    Stage Count;
    Stage CountReduce;
    Stage Scan;
    Stage ScanAdd;
    Stage Scatter;
    Stage ScatterPayload;

    // For now, copied over from sample
    VkDescriptorSetLayout m_SortDescriptorSetLayoutInputOutputs;
    VkDescriptorSetLayout m_SortDescriptorSetLayoutScan;
    VkDescriptorSetLayout m_SortDescriptorSetLayoutScratch;

    VkDescriptorSet m_SortDescriptorSetInputOutput[2];