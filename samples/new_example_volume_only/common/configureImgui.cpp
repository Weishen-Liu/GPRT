#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "./configureImgui.hpp"

void
ConfigureImgui::initLight() {
  AmbientLight sampleAmbientLight;
  std::string lightName;
  lightName = "Ambient Light " + std::to_string(ambientLightUniqueName);
  sampleAmbientLight.name = lightName.c_str();
  ambientLightUniqueName++;
  sampleAmbientLight.intensity = float3(1.f, 1.f, 1.f);
  sampleAmbientLight.choosed = false;
  LIST_OF_AMBIENT_LIGHTS.push_back(sampleAmbientLight);

  DirectionalLight sampleDirectionalLight;
  lightName = "Directional Light " + std::to_string(directionalLightUniqueName);
  sampleDirectionalLight.name = lightName.c_str();
  directionalLightUniqueName++;
  sampleDirectionalLight.intensity = float3(1.f, 1.f, 1.f);
  sampleDirectionalLight.direction = float3(0.f, -5.f, 0.f);
  sampleDirectionalLight.choosed = false;
  LIST_OF_DIRECTIONAL_LIGHTS.push_back(sampleDirectionalLight);

  for (auto &each_light : LIST_OF_AMBIENT_LIGHTS) {
    if (each_light.choosed) {
      SELECTED_LIGHTS++;
    }
  }
  for (auto &each_light : LIST_OF_DIRECTIONAL_LIGHTS) {
    if (each_light.choosed) {
      SELECTED_LIGHTS++;
    }
  }
}

void
ConfigureImgui::initVolume() {
  std::cout << "Init Volume" << std::endl;
  for (int i = 0; i < INITIAL_VOLUME.size(); i++) {
    owl::common::LCG<4> random;
    random.init(0, 1);

    Volume newVolume;
    newVolume.name = INITIAL_VOLUME[i]->name;
    newVolume.path = INITIAL_VOLUME[i]->path;
    LIST_OF_VOLUMES.push_back(newVolume);
  }

  for (auto &each_volume : LIST_OF_VOLUMES) {
    for (auto &each_instance : each_volume.instances) {
      if (each_instance.choosed) {
        each_volume.SELECTED_INSTANCE++;
      }
    }
  }
}

template <typename T>
void
ConfigureImgui::createDefaultInstance(T &target) {
  for (int i = 0; i < target.generateInstance; i++) {
    Instance instance;
    instance.translate = target.defaultTranslate;
    instance.scale = target.defaultScale;
    instance.rotate = target.defaultRotate;
    instance.name = target.name + " " + std::to_string(target.instanceUniqueName);
    target.instanceUniqueName++;
    target.instances.push_back(instance);
  }
  if (typeid(T) == typeid(Volume)) {
    updateVolumeSelection = true;
  }
}

void
ConfigureImgui::addVolumeInstance(Volume &volume) {
  if (volume.addInstanceWindow) {
    const char *item = volume.name.c_str();
    std::string windowName = "Instances for " + volume.name;
    ImGui::Begin(windowName.c_str(), &volume.addInstanceWindow);
    ImGui::Text("Generate");
    ImGui::SameLine();
    std::string generateInstanceName = " Instance for " + volume.name;
    ImGui::SliderInt(generateInstanceName.c_str(), &volume.generateInstance, 1, 3);
    if (ImGui::Button("Create Volume Instances")) {
      createDefaultInstance<Volume>(volume);
    }
    ImGui::Text("");
    ImGui::Text("Selected %d Instances under Volume '%s'", volume.SELECTED_INSTANCE, item);
    for (int eachInstance = 0; eachInstance < volume.instances.size(); eachInstance++) {
      const char *instance_name = volume.instances[eachInstance].name.c_str();
      if (ImGui::Checkbox(instance_name, &volume.instances[eachInstance].choosed)) {
        updateVolumeSelection = true;
        if (volume.instances[eachInstance].choosed == false) {
          if (volume.current_instance == instance_name) {
            volume.current_instance = NULL;
            volume.current_instance_index = -1;
          }
          volume.SELECTED_INSTANCE -= 1;
        } else {
          volume.SELECTED_INSTANCE += 1;
        }
      }
    }
    ImGui::End();
  }
}

void
ConfigureImgui::volumeInstances() {
  for (int eachVolume = 0; eachVolume < LIST_OF_VOLUMES.size(); eachVolume++) {
    const char *item = LIST_OF_VOLUMES[eachVolume].name.c_str();
    if (LIST_OF_VOLUMES[eachVolume].openInstanceWindow) {
      ImGui::Begin(item, &LIST_OF_VOLUMES[eachVolume].openInstanceWindow);
      // Add Instance Window
      if (ImGui::Button("Add Volume Instance")) {
        LIST_OF_VOLUMES[eachVolume].addInstanceWindow = !LIST_OF_VOLUMES[eachVolume].addInstanceWindow;
      }
      addVolumeInstance(LIST_OF_VOLUMES[eachVolume]);

      // Configure Instance Translate
      ImGui::Text("Loaded ");
      ImGui::SameLine();
      if (ImGui::BeginCombo(" Instance",
                            LIST_OF_VOLUMES[eachVolume].current_instance))   // The second parameter is the label
                                                                             // previewed before opening the combo.
      {
        for (int n = 0; n < LIST_OF_VOLUMES[eachVolume].instances.size(); n++) {
          const char *isntance_name = LIST_OF_VOLUMES[eachVolume].instances[n].name.c_str();
          bool is_selected =
              (LIST_OF_VOLUMES[eachVolume].current_instance ==
               isntance_name);   // You can store your selection however you want, outside or inside your objects
          if (LIST_OF_VOLUMES[eachVolume].instances[n].choosed == false) {
            continue;
          }
          if (ImGui::Selectable(isntance_name, is_selected)) {
            LIST_OF_VOLUMES[eachVolume].current_instance = isntance_name;
            LIST_OF_VOLUMES[eachVolume].current_instance_index = n;
          }
          if (is_selected) {
            ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for
                                            // keyboard navigation support)
          }
        }
        ImGui::EndCombo();
      }
      if (LIST_OF_VOLUMES[eachVolume].current_instance != NULL) {
        updateTransform(LIST_OF_VOLUMES[eachVolume]);
      }
      ImGui::End();
    }
  }
}

void
restrictRange(float min_range, float max_range, float &input) {
  if (input > max_range)
    input = max_range;
  if (input < min_range)
    input = min_range;
}

void
ConfigureImgui::inputAndSlider(float3 &source, float min_v, float max_v, const char *title, const char *inputLabel,
                               const char *sliderLabel, bool &trigger) {
  float source3[3] = {source.x, source.y, source.z};
  ImGui::Text(title);
  if (ImGui::InputFloat3(inputLabel, source3)) {
    restrictRange(min_v, max_v, source3[0]);
    restrictRange(min_v, max_v, source3[1]);
    restrictRange(min_v, max_v, source3[2]);

    source.x = source3[0];
    source.y = source3[1];
    source.z = source3[2];
    trigger = true;
  }

  if (ImGui::SliderFloat3(sliderLabel, source3, min_v, max_v)) {
    source.x = source3[0];
    source.y = source3[1];
    source.z = source3[2];
    trigger = true;
  }
}

void
ConfigureImgui::updateTransform(Volume &volume) {
  inputAndSlider(volume.instances[volume.current_instance_index].translate, -100000.f, 100000.f, "Translate",
                 "Translate Input", "Translate Slider", updateVolumeTranslate);
  // Current no use cases
  inputAndSlider(volume.instances[volume.current_instance_index].scale, 1.f, 100000.f, "Scale", "Scale Input",
                 "Scale Slider", updateVolumeScale);
  inputAndSlider(volume.instances[volume.current_instance_index].rotate, -10.f, 10.f, "Rotate", "Rotate Input",
                 "Rotate Slider", updateVolumeRotate);
}

void
ConfigureImgui::renderVolumeCP() {
  if (showVolumeControlPanel) {
    ImGui::Begin("Volume Control Panel", &showVolumeControlPanel);

    for (int i = 0; i < LIST_OF_VOLUMES.size(); i++) {
      const char *item = LIST_OF_VOLUMES[i].name.c_str();
      if (ImGui::Button(item)) {
        LIST_OF_VOLUMES[i].openInstanceWindow = !LIST_OF_VOLUMES[i].openInstanceWindow;
      }
    }
    volumeInstances();
    ImGui::End();
  }
}

void
ConfigureImgui::updateLight() {
  if (current_light.type == ALL_LIGHTS[0].c_str()) {
    inputAndSlider(LIST_OF_AMBIENT_LIGHTS[current_light.index].intensity, 0.f, 5.f, "Intensity",
                   "Ambient Intensity Input", "Ambient Intensity Slider", updateLights);
  } else if (current_light.type == ALL_LIGHTS[1].c_str()) {
    inputAndSlider(LIST_OF_DIRECTIONAL_LIGHTS[current_light.index].intensity, 0.f, 100.f, "Intensity",
                   "Directional Intensity Input", "Directional Intensity Slider", updateLights);

    inputAndSlider(LIST_OF_DIRECTIONAL_LIGHTS[current_light.index].direction, -1000.f, 1000.f, "Direction",
                   "Directional Direction Input", "Directional Direction Slider", updateLights);
  }
}

void
ConfigureImgui::addLight() {
  if (addNewLight) {
    ImGui::Begin("Add Light", &addNewLight);
    if (ImGui::Button("Add New Ambient Light")) {
      AmbientLight sampleAmbientLight;
      sampleAmbientLight.name = "Ambient Light " + std::to_string(ambientLightUniqueName);
      ambientLightUniqueName++;
      sampleAmbientLight.intensity = float3(1.f, 1.f, 1.f);
      sampleAmbientLight.choosed = false;
      LIST_OF_AMBIENT_LIGHTS.push_back(sampleAmbientLight);
    }

    if (ImGui::Button("Add New Directional Light")) {
      DirectionalLight sampleDirectionalLight;
      sampleDirectionalLight.name = "Directional Light " + std::to_string(directionalLightUniqueName);
      directionalLightUniqueName++;
      sampleDirectionalLight.intensity = float3(1.f, 1.f, 1.f);
      sampleDirectionalLight.direction = float3(0.f, -5.f, 0.f);
      sampleDirectionalLight.choosed = false;
      LIST_OF_DIRECTIONAL_LIGHTS.push_back(sampleDirectionalLight);
    }

    ImGui::Text("Selected %d Lights", SELECTED_LIGHTS);
    for (int i = 0; i < LIST_OF_AMBIENT_LIGHTS.size(); i++) {
      const char *selectedLight = LIST_OF_AMBIENT_LIGHTS[i].name.c_str();
      if (ImGui::Checkbox(selectedLight, &LIST_OF_AMBIENT_LIGHTS[i].choosed)) {
        updateLights = true;
        if (LIST_OF_AMBIENT_LIGHTS[i].choosed == false) {
          if (current_light.name == selectedLight) {
            current_light.name = NULL;
            current_light.type = NULL;
            current_light.index = -1;
          }
          SELECTED_LIGHTS -= 1;
        } else {
          SELECTED_LIGHTS += 1;
        }
      }
    }
    for (int i = 0; i < LIST_OF_DIRECTIONAL_LIGHTS.size(); i++) {
      const char *selectedLight = LIST_OF_DIRECTIONAL_LIGHTS[i].name.c_str();
      if (ImGui::Checkbox(selectedLight, &LIST_OF_DIRECTIONAL_LIGHTS[i].choosed)) {
        updateLights = true;
        if (LIST_OF_DIRECTIONAL_LIGHTS[i].choosed == false) {
          if (current_light.name == selectedLight) {
            current_light.name = NULL;
            current_light.type = NULL;
            current_light.index = -1;
          }
          SELECTED_LIGHTS -= 1;
        } else {
          SELECTED_LIGHTS += 1;
        }
      }
    }
    ImGui::End();
  }
}

void
ConfigureImgui::renderLightCP() {
  if (showLightControlPanel) {
    ImGui::Begin("Light Control Panel", &showLightControlPanel);

    if (ImGui::Button("Add New Light")) {
      addNewLight = !addNewLight;
    }
    addLight();

    ImGui::Text("Loaded ");
    ImGui::SameLine();
    if (ImGui::BeginCombo(
            " Light", current_light.name))   // The second parameter is the label previewed before opening the combo.
    {
      for (int n = 0; n < LIST_OF_AMBIENT_LIGHTS.size(); n++) {
        const char *light = LIST_OF_AMBIENT_LIGHTS[n].name.c_str();
        bool is_selected = (current_light.name ==
                            light);   // You can store your selection however you want, outside or inside your objects
        if (LIST_OF_AMBIENT_LIGHTS[n].choosed == false) {
          continue;
        }
        if (ImGui::Selectable(light, is_selected)) {
          current_light.name = light;
          current_light.index = n;
          current_light.type = ALL_LIGHTS[0].c_str();
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for
                                          // keyboard navigation support)
        }
      }
      for (int n = 0; n < LIST_OF_DIRECTIONAL_LIGHTS.size(); n++) {
        const char *light = LIST_OF_DIRECTIONAL_LIGHTS[n].name.c_str();
        bool is_selected = (current_light.name ==
                            light);   // You can store your selection however you want, outside or inside your objects
        if (LIST_OF_DIRECTIONAL_LIGHTS[n].choosed == false) {
          continue;
        }
        if (ImGui::Selectable(light, is_selected)) {
          current_light.name = light;
          current_light.index = n;
          current_light.type = ALL_LIGHTS[1].c_str();
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for
                                          // keyboard navigation support)
        }
      }
      ImGui::EndCombo();
    }
    if (current_light.name != NULL) {
      updateLight();
    }
    ImGui::End();
  }
}

void
ConfigureImgui::render() {
  // render your GUI
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::SetNextWindowSizeConstraints(ImVec2(300, 300), ImVec2(FLT_MAX, FLT_MAX));
  if (showImgui) {
    ImGui::Begin("Control Panel", &showImgui);
    ImGui::Text("io.WantCaptureMouse: %d", ImGui::GetIO().WantCaptureMouse);

    if (ImGui::Button("Volume Control Panel")) {
      showVolumeControlPanel = !showVolumeControlPanel;
    }

    if (ImGui::Button("Light Control Panel")) {
      showLightControlPanel = !showLightControlPanel;
    }
    renderVolumeCP();
    renderLightCP();
  }

  ImGui::End();

  // Render dear imgui into screen
  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}