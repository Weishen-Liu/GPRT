#include "jsonLoader.hpp"

std::string check_valid_extension(std::string filename)
{
  const auto ext = filename.substr(filename.find_last_of(".") + 1);
  if (ext == "json") return "json";
  throw std::runtime_error("unknown scene format");
}

void extract_root_and_workdir(std::string &file_path, json &root, std::string &workdir)
{
  std::ifstream file(file_path);
  std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  std::filesystem::path p = file_path;
  workdir = p.remove_filename().string();
  workdir = workdir.empty() ? "." : workdir; // make sure workdir is never empty

  root = json::parse(text, nullptr, true, true);
  assert(root.is_object());
}

void reverse_byte_order(char* data, size_t elemCount, size_t elemSize)
{
  switch (elemSize) {
  case 1: break;
  case 2:
    for (size_t i = 0; i < elemCount; ++i)
      swap_bytes<2>(&data[i * elemSize]);
    break;
  case 4:
    for (size_t i = 0; i < elemCount; ++i)
      swap_bytes<4>(&data[i * elemSize]);
    break;
  case 8:
    for (size_t i = 0; i < elemCount; ++i)
      swap_bytes<8>(&data[i * elemSize]);
    break;
  default: assert(false);
  }
}

array_1d_float4_t
CreateArray1DFloat4(const std::vector<float4>& input, bool shared)
{
  return CreateArray1DScalar(input, shared);
}

array_1d_float4_t
CreateArray1DFloat4(const float4* input, size_t len, bool shared)
{
  return CreateArray1DScalar(input, len, shared);
}

array_1d_float4_t
CreateColorMap(const std::string& name)
{
  if (colormap::data.count(name) > 0) {
    // auto& _arr = *colormap::data.at(name);
    // std::vector<float4> arr;
    // for (int i = 0; i < _arr.size(); ++i) {
    //   arr.push_back(float4(_arr[i].r, _arr[i].g, _arr[i].b, _arr[i].a));
    // }
    std::vector<float4>& arr = *((std::vector<float4>*)colormap::data.at(name));
    return CreateArray1DFloat4(arr, false);
  }
  else {
    throw std::runtime_error("Unexpected colormap name: " + name);
  }
}

array_3d_scalar_t CreateArray3DScalarFromFile(const std::string& filename, int3 dims, ValueType type, size_t offset, bool is_big_endian)
{
  // data geometry
  assert(dims.x > 0 && dims.y > 0 && dims.z > 0);

  size_t elem_count = (size_t)dims.x * dims.y * dims.z;

  size_t elem_size = // clang-format off
    (type == VALUE_TYPE_UINT8  || type == VALUE_TYPE_INT8 ) ? sizeof(uint8_t)  :
    (type == VALUE_TYPE_UINT16 || type == VALUE_TYPE_INT16) ? sizeof(uint16_t) :
    (type == VALUE_TYPE_UINT32 || type == VALUE_TYPE_INT32 || type == VALUE_TYPE_FLOAT) ? 
    sizeof(uint32_t) : sizeof(double);
  // clang-format on

  size_t data_size = elem_count * elem_size;

  // load the data
  std::shared_ptr<char[]> data_buffer;

  std::ifstream ifs(filename, std::ios::in | std::ios::binary);
  if (ifs.fail()) // cannot open the file
  {
    throw std::runtime_error("Cannot open the file");
  }

  ifs.seekg(0, std::ios::end);
  size_t file_size = ifs.tellg();
  if (file_size < offset + data_size) // file size does not match data size
  {
    throw std::runtime_error("File size does not match data size");
  }
  ifs.seekg(offset, std::ios::beg);

  try {
    data_buffer.reset(new char[data_size]);
  }
  catch (std::bad_alloc&) // memory allocation failed
  {
    throw std::runtime_error("Cannot allocate memory for the data");
  }

  // read data
  ifs.read(data_buffer.get(), data_size);
  if (ifs.fail()) // reading data failed
  {
    throw std::runtime_error("Cannot read the file");
  }

  // reverse byte-order if necessary
  const bool reverse = (is_big_endian && elem_size > 1);
  if (reverse) {
    reverse_byte_order(data_buffer.get(), elem_count, elem_size);
  }

  ifs.close();

  // finalize
  array_3d_scalar_t output = std::make_shared<Array<3>>();
  output->dims.x = dims.x;
  output->dims.y = dims.y;
  output->dims.z = dims.z;

  output->type = type;
  output->acquire_data(std::move(data_buffer));

  return output;
}

void create_scene_volume(json &jsdata, std::string &workdir, Volume &volume)
{
  const auto format = jsdata[FORMAT].get<std::string>();

  if (format == REGULAR_GRID_RAW_BINARY) {
    auto filename = jsdata[FILE_NAME].get<std::string>();
    {
      std::filesystem::path path(filename);
      if (path.is_relative()) filename = workdir + "/" + filename;
    }
    auto dims = scalar_from_json<int3>(jsdata[DIMENSIONS]);
    auto type = scalar_from_json<ValueType>(jsdata[TYPE]);
    auto offset = scalar_from_json<size_t>(jsdata, OFFSET, 0);
    auto flipped = scalar_from_json<bool>(jsdata, FILE_UPPER_LEFT, false);
    auto is_big_endian = scalar_from_json<Endianness>(jsdata, ENDIAN, OVR_LITTLE_ENDIAN) == OVR_BIG_ENDIAN;

    volume.data = CreateArray3DScalarFromFile(filename, dims, type, offset, is_big_endian);
    volume.grid_origin = float3(0.f, 0.f, 0.f);
  }
  else {
    throw std::runtime_error("data type unimplemented");
  }
}

float2 range_from_json(json jsrange)
{
  if (!jsrange.contains("minimum") || !jsrange.contains("maximum")) return float2(0.f, 0.f);
  return float2(jsrange["minimum"].get<float>(), jsrange["maximum"].get<float>());
}

void create_scene_tfn(const json& jsview, Volume &volume)
{
  const auto& jstfn = jsview[VOLUME][TRANSFER_FUNCTION];
  const auto& jsvolume = jsview[VOLUME];

  tfn::TransferFunctionCore tf;
  tfn::loadTransferFunction(jstfn, tf);

  auto* table = (float4*)tf.data();
  std::vector<float4> color(tf.resolution());
  std::vector<float> alpha(tf.resolution());
  for (int i = 0; i < tf.resolution(); ++i) {
    auto rgba = table[i];
    color[i] = float4(rgba.xyz(), 1.f);
    alpha[i] = rgba.w;
  }
  if (alpha[0] < 0.01f) alpha[0] = 0.f;
  if (alpha[tf.resolution()-1] < 0.01f) alpha[tf.resolution()-1] = 0.f;

  volume.transferFunction.color   = CreateArray1DFloat4(color, false);
  volume.transferFunction.opacity = CreateArray1DScalar(alpha, false);

  if (jsvolume.contains(SCALAR_MAPPING_RANGE_UNNORMALIZED)) {
    auto r = range_from_json(jsvolume[SCALAR_MAPPING_RANGE_UNNORMALIZED]);
    volume.transferFunction.value_range.x = r.x;
    volume.transferFunction.value_range.y = r.y;
  }

  /* try it ... */
  else if (jsvolume.contains(SCALAR_MAPPING_RANGE)) {
    auto r = range_from_json(jsvolume[SCALAR_MAPPING_RANGE]);
    switch (volume.data->type) {
    case VALUE_TYPE_UINT8:
      volume.transferFunction.value_range.x = std::numeric_limits<uint8_t>::max() * r.x;
      volume.transferFunction.value_range.y = std::numeric_limits<uint8_t>::max() * r.y;
      break;
    case VALUE_TYPE_INT8:
      volume.transferFunction.value_range.x = std::numeric_limits<int8_t>::max() * r.x;
      volume.transferFunction.value_range.y = std::numeric_limits<int8_t>::max() * r.y;
      break;
    case VALUE_TYPE_UINT16:
      volume.transferFunction.value_range.x = std::numeric_limits<uint16_t>::max() * r.x;
      volume.transferFunction.value_range.y = std::numeric_limits<uint16_t>::max() * r.y;
      break;
    case VALUE_TYPE_INT16:
      volume.transferFunction.value_range.x = std::numeric_limits<int16_t>::max() * r.x;
      volume.transferFunction.value_range.y = std::numeric_limits<int16_t>::max() * r.y;
      break;
    case VALUE_TYPE_UINT32:
      volume.transferFunction.value_range.x = std::numeric_limits<uint32_t>::max() * r.x;
      volume.transferFunction.value_range.y = std::numeric_limits<uint32_t>::max() * r.y;
      break;
    case VALUE_TYPE_INT32:
      volume.transferFunction.value_range.x = std::numeric_limits<int32_t>::max() * r.x;
      volume.transferFunction.value_range.y = std::numeric_limits<int32_t>::max() * r.y;
      break;
    case VALUE_TYPE_FLOAT:
    case VALUE_TYPE_DOUBLE:
      volume.transferFunction.value_range.x = r.x;
      volume.transferFunction.value_range.y = r.y;
      break;
    default: throw std::runtime_error("unknown data type");
    }
  }

  else {
    /* calculate the volume value range ... */
    throw std::runtime_error("unknown data range");
  }
}

void load_volume_data_from_file(Volume &volume)
{
    // Open JSON File
    std::string file_path = volume.path;
    std::string file_type = check_valid_extension(file_path);
    json root;
    std::string workdir;
    extract_root_and_workdir(file_path, root, workdir);

    if (file_type == "json") {
      for (auto& ds : root[DATA_SOURCE]) {
        create_scene_volume(ds, workdir, volume);

        create_scene_tfn(root[VIEW], volume);

        if (!root[VIEW][VOLUME].contains(SCALAR_MAPPING_RANGE_UNNORMALIZED)) {
          auto type = scalar_from_json<ValueType>(ds[TYPE]);
          if (type != VALUE_TYPE_FLOAT && type != VALUE_TYPE_DOUBLE) {
            std::cerr << "[vidi3d] An unnormalized value range cannot be found for "
                        "transfer function, incorrect results can be produced."
                      << std::endl;
          }
        }
      }
    }
}