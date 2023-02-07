#pragma once
#ifndef INCLUDE_NEWEXAMPLE_VOLUME_COMMON_ARRAY
#define INCLUDE_NEWEXAMPLE_VOLUME_COMMON_ARRAY

#include <vector>
#include <gprt.h>
#include <GLFW/glfw3.h>
#include "math.h"
#include <colormap.h>
#include "tfn/json.h"

using json = nlohmann::json;

#define VOLUME "volume"
#define TRANSFER_FUNCTION "transferFunction"
#define SCALAR_MAPPING_RANGE_UNNORMALIZED "scalarMappingRangeUnnormalized"
#define SCALAR_MAPPING_RANGE "scalarMappingRange"
#define FORMAT "format"
#define REGULAR_GRID_RAW_BINARY "REGULAR_GRID_RAW_BINARY"
#define FILE_NAME "fileName"
#define DIMENSIONS "dimensions"
#define TYPE "type"
#define OFFSET "offset"
#define FILE_UPPER_LEFT "fileUpperLeft"
#define ENDIAN "endian"
#define EYE "eye"
#define CENTER "center"
#define UP "up"
#define FOVY "fovy"
#define CAMERA "camera"
#define DATA_SOURCE "dataSource"
#define VIEW "view"
#define POSITION "position"
#define LIGHT_SOURCE "lightSource"
#define SAMPLING_DISTANCE "sampleDistance"

enum ValueType {
  VALUE_TYPE_UINT8 = 100,
  VALUE_TYPE_INT8,

  VALUE_TYPE_UINT16 = 200,
  VALUE_TYPE_INT16,

  VALUE_TYPE_UINT32 = 300,
  VALUE_TYPE_INT32,

  VALUE_TYPE_FLOAT = 400,
  VALUE_TYPE_FLOAT2,
  VALUE_TYPE_FLOAT3,
  VALUE_TYPE_FLOAT4,

  VALUE_TYPE_DOUBLE = 500,
  VALUE_TYPE_DOUBLE2,
  VALUE_TYPE_DOUBLE3,
  VALUE_TYPE_DOUBLE4,

  VALUE_TYPE_VOID = 1000,
};

inline int
value_type_size(ValueType type)
{
  switch (type) {
  case VALUE_TYPE_UINT8: return sizeof(uint8_t);
  case VALUE_TYPE_INT8: return sizeof(int8_t);
  case VALUE_TYPE_UINT16: return sizeof(uint16_t);
  case VALUE_TYPE_INT16: return sizeof(int16_t);
  case VALUE_TYPE_UINT32: return sizeof(uint32_t);
  case VALUE_TYPE_INT32: return sizeof(int32_t);
  case VALUE_TYPE_FLOAT: return sizeof(float);
  case VALUE_TYPE_DOUBLE: return sizeof(double);

  case VALUE_TYPE_FLOAT2: return sizeof(float2);
  case VALUE_TYPE_FLOAT3: return sizeof(float3);
  case VALUE_TYPE_FLOAT4: return sizeof(float4);

  default: throw std::runtime_error("unknown type encountered");
  }
}

template<typename T>
ValueType
value_type();

#define ovr_instantiate_value_type_function(TYPE, type) \
  template<>                                            \
  inline ValueType value_type<type>()                   \
  {                                                     \
    return TYPE;                                        \
  }
/* clang-format off */
ovr_instantiate_value_type_function(VALUE_TYPE_UINT8,  uint8_t);
ovr_instantiate_value_type_function(VALUE_TYPE_INT8,   int8_t);
ovr_instantiate_value_type_function(VALUE_TYPE_UINT16, uint16_t);
ovr_instantiate_value_type_function(VALUE_TYPE_INT16,  int16_t);
ovr_instantiate_value_type_function(VALUE_TYPE_UINT32, uint32_t);
ovr_instantiate_value_type_function(VALUE_TYPE_INT32,  int32_t);
ovr_instantiate_value_type_function(VALUE_TYPE_FLOAT,  float);
ovr_instantiate_value_type_function(VALUE_TYPE_FLOAT2, float2);
ovr_instantiate_value_type_function(VALUE_TYPE_FLOAT3, float3);
ovr_instantiate_value_type_function(VALUE_TYPE_FLOAT4, float4);
ovr_instantiate_value_type_function(VALUE_TYPE_DOUBLE, double);

// ------------------------------------------------------------------
// Array Definitions
// ------------------------------------------------------------------

template<int DIM>
struct Array {
  enum { VECTOR_DIMENSION = DIM };

  ValueType type;
  math::vec_t<int, DIM> dims{ 0 };

  Array() {}
  ~Array() {}

  size_t size()
  {
    if (DIM == 0) return 0;
    size_t size = 1;
    for (int i = 0; i < DIM; i++)
      size *= dims[i];

    return size;
  }

  void allocate(void* ptr = nullptr)
  {
    owned_buffer.reset(new char[dims.long_product() * value_type_size(type)]);
    buffer = owned_buffer.get();
    if (ptr)
      memcpy(buffer, ptr, dims.long_product() * value_type_size(type));
  }

  void set_data(void* ptr)
  {
    owned_buffer.reset();
    buffer = (char*)ptr;
  }

  void acquire_data(std::shared_ptr<char[]> ptr)
  {
    owned_buffer = ptr;
    buffer = owned_buffer.get();
  }

  char* data()
  {
    return buffer;
  }

  char* data() const
  {
    return buffer;
  }

  template<typename T>
  T* data_typed()
  {
    if (type != value_type<T>())
      throw std::runtime_error("mismatched type!");

    return (T*)buffer;
  }

  template<typename T>
  T* data_typed() const
  {
    if (type != value_type<T>())
      throw std::runtime_error("mismatched type!");

    return (T*)buffer;
  }

private:
  char* buffer{ nullptr };
  std::shared_ptr<char[]> owned_buffer;
};

using Array1DScalar = Array<1>;
using Array1DFloat4 = Array<1>;
using Array3DScalar = Array<3>;
using array_1d_scalar_t = std::shared_ptr<Array1DScalar>;
using array_1d_float4_t = std::shared_ptr<Array1DFloat4>;
using array_3d_scalar_t = std::shared_ptr<Array3DScalar>;

NLOHMANN_JSON_SERIALIZE_ENUM(ValueType, {
  { ValueType::VALUE_TYPE_INT8, "BYTE" },
  { ValueType::VALUE_TYPE_UINT8, "UNSIGNED_BYTE" },
  { ValueType::VALUE_TYPE_INT16, "SHORT" },
  { ValueType::VALUE_TYPE_UINT16, "UNSIGNED_SHORT" },
  { ValueType::VALUE_TYPE_INT32, "INT" },
  { ValueType::VALUE_TYPE_UINT32, "UNSIGNED_INT" },
  { ValueType::VALUE_TYPE_FLOAT, "FLOAT" },
  { ValueType::VALUE_TYPE_DOUBLE, "DOUBLE" },
}); // clang-format on

enum Endianness { OVR_LITTLE_ENDIAN, OVR_BIG_ENDIAN };
NLOHMANN_JSON_SERIALIZE_ENUM(Endianness, {
  { OVR_LITTLE_ENDIAN, "LITTLE_ENDIAN" },
  { OVR_BIG_ENDIAN, "BIG_ENDIAN" },
}); // clang-format on

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(int2, x, y);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(int3, x, y, z);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(int4, x, y, z, w); 
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(float2, x, y);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(float3, x, y, z);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(float4, x, y, z, w); 

template<typename ScalarT>
inline ScalarT
scalar_from_json(const json& in);

#define define_scalar_serialization(T) template<> inline T scalar_from_json<T>(const json& in) { return in.get<T>(); }
define_scalar_serialization(std::string);
define_scalar_serialization(bool);
define_scalar_serialization(int64_t);
define_scalar_serialization(uint64_t);
#ifdef __APPLE__
define_scalar_serialization(size_t);
#endif
define_scalar_serialization(double);

template<typename ScalarT/*, typename std::enable_if_t<!std::is_arithmetic<ScalarT>::value> = true*/>
inline ScalarT
scalar_from_json(const json& in)
{
  ScalarT v;
  from_json(in, v);
  return v;
}

template<typename ScalarT>
inline ScalarT
scalar_from_json(const json& in, const std::string& key)
{
  if (!in.is_object()) {
    throw std::runtime_error("has to be a JSON object");
  }
  if (!in.contains(key)) {
    throw std::runtime_error("incorrect key: " + key);
  }
  return scalar_from_json<ScalarT>(in[key]);
}

template<typename ScalarT>
inline ScalarT
scalar_from_json(const json& in, const std::string& key, const ScalarT& value)
{
  if (!in.is_object()) {
    throw std::runtime_error("has to be a JSON object");
  }
  if (in.contains(key)) {
    return scalar_from_json<ScalarT>(in[key]);
  }
  else {
    return value;
  }
}

template<size_t Size>
inline void
swap_bytes(void* data)
{
  char* p = reinterpret_cast<char*>(data);
  char* q = p + Size - 1;
  while (p < q)
    std::swap(*(p++), *(q--));
}

template<>
inline void
swap_bytes<1>(void*)
{
}

template<>
inline void
swap_bytes<2>(void* data)
{
  char* p = reinterpret_cast<char*>(data);
  std::swap(p[0], p[1]);
}

template<>
inline void
swap_bytes<4>(void* data)
{
  char* p = reinterpret_cast<char*>(data);
  std::swap(p[0], p[3]);
  std::swap(p[1], p[2]);
}

template<>
inline void
swap_bytes<8>(void* data)
{
  char* p = reinterpret_cast<char*>(data);
  std::swap(p[0], p[7]);
  std::swap(p[1], p[6]);
  std::swap(p[2], p[5]);
  std::swap(p[3], p[4]);
}

template<typename T>
inline void
swap_bytes(T* data)
{
  swap_bytes<sizeof(T)>(reinterpret_cast<void*>(data));
}

template<typename T>
array_1d_scalar_t
CreateArray1DScalar(const std::vector<T>& input, bool shared)
{
  array_1d_scalar_t output = std::make_shared<Array<1>>();

  output->type = value_type<T>();
  output->dims = input.size();
  if (shared)
    output->set_data((void*)input.data());
  else
    output->allocate((void*)input.data());

  return output;
}
#define instantiate_create_array1dscalar_vector(T) \
  template array_1d_scalar_t CreateArray1DScalar<T>(const std::vector<T>& input, bool shared);
instantiate_create_array1dscalar_vector(uint8_t);
instantiate_create_array1dscalar_vector(int8_t);
instantiate_create_array1dscalar_vector(uint32_t);
instantiate_create_array1dscalar_vector(int32_t);
instantiate_create_array1dscalar_vector(float);
instantiate_create_array1dscalar_vector(double);
#undef instantiate_create_array1dscalar_vector

template<typename T>
array_1d_scalar_t
CreateArray1DScalar(const T* input, size_t len, bool shared)
{
  array_1d_scalar_t output = std::make_shared<Array<1>>();

  output->type = value_type<T>();
  output->dims = len;
  if (shared)
    output->set_data((void*)input);
  else
    output->allocate((void*)input);

  return output;
}
#define instantiate_create_array1dscalar_pointer(T) \
  template array_1d_scalar_t CreateArray1DScalar<T>(const T* input, size_t len, bool shared);
instantiate_create_array1dscalar_pointer(uint8_t);
instantiate_create_array1dscalar_pointer(int8_t);
instantiate_create_array1dscalar_pointer(uint32_t);
instantiate_create_array1dscalar_pointer(int32_t);
instantiate_create_array1dscalar_pointer(float);
instantiate_create_array1dscalar_pointer(double);
#undef instantiate_create_array1dscalar_pointer

#endif
