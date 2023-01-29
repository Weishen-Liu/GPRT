#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl2.h>

#include "./configureImgui.hpp"

void ConfigureImgui::initObj()
{
    std::cout<<"Init Obj"<<std::endl;
    for (int i = 0; i < INITIAL_OBJ.size(); i++)
    {
        owl::common::LCG<4> random;
        random.init(0, 1);
        float3 lambertian_color = float3(random(), random(), random());
        float3 metal_color = float3(random(), random(), random());
        float metal_fuzz = random();
        float dielectric_ref_idx = random();

        Obj newObj;
        newObj.name = INITIAL_OBJ[i]->name;
        newObj.path = INITIAL_OBJ[i]->path;
        newObj.transform = float3(0.0f, 0.0f, 0.0f);
        newObj.material.type = ALL_MATERIALS[0].c_str();
        newObj.material.lambertian.albedo = lambertian_color;
        newObj.material.metal.albedo = metal_color;
        newObj.material.metal.fuzz = metal_fuzz;
        newObj.material.dielectric.ref_idx = dielectric_ref_idx;

        LIST_OF_OBJS.push_back(newObj);
    }

    for (auto each_obj: LIST_OF_OBJS) {
        if (each_obj.choosed) {
            SELECTED_OBJS++;
        }
    }
}

void ConfigureImgui::initLight()
{
    AmbientLight sampleAmbientLight;
    sampleAmbientLight.name = "Ambient Light Sample #1";
    sampleAmbientLight.intensity = float3(1.f, 1.f, 1.f);
    sampleAmbientLight.choosed = false;
    LIST_OF_AMBIENT_LIGHTS.push_back(sampleAmbientLight);

    DirectionalLight sampleDirectionalLight;
    sampleDirectionalLight.name = "Directional Light Sample #1";
    sampleDirectionalLight.intensity = float3(1.f, 1.f, 1.f);
    sampleDirectionalLight.direction = float3(0.f, -5.f, 0.f);
    sampleDirectionalLight.choosed = true;
    LIST_OF_DIRECTIONAL_LIGHTS.push_back(sampleDirectionalLight);

    for (auto each_light: LIST_OF_AMBIENT_LIGHTS) {
        if (each_light.choosed) {
            SELECTED_LIGHTS++;
        }
    }
    for (auto each_light: LIST_OF_DIRECTIONAL_LIGHTS) {
        if (each_light.choosed) {
            SELECTED_LIGHTS++;
        }
    }
}

void ConfigureImgui::addObj()
{
    if (addNewObj)
    {
        ImGui::Begin("Add Obj", &addNewObj);
        ImGui::Text("Selected %d Objs", SELECTED_OBJS);
        for (int i = 0; i < LIST_OF_OBJS.size(); i++)
        {
            const char* item = LIST_OF_OBJS[i].name.c_str();
            if (ImGui::Checkbox(item, &LIST_OF_OBJS[i].choosed))
            {
                updateObjSelection = true;
                if (LIST_OF_OBJS[i].choosed == false) {
                    if (current_item == item)
                    {
                        current_item = NULL;
                        current_item_index = -1;
                    }
                    SELECTED_OBJS -= 1;
                } else {
                    SELECTED_OBJS += 1;
                }
            }
        }
        ImGui::End();
    }
}

void restrictRange(float min_range, float max_range, float &input)
{
    if (input > max_range) input = max_range;
    if (input < min_range) input = min_range;
}

void ConfigureImgui::inputAndSlider(float3& source, float min_v, float max_v, const char *title, const char *inputLabel, const char *sliderLabel, bool& trigger)
{
    float source3[3] = {source.x, source.y, source.z};
    ImGui::Text(title);
    if (ImGui::InputFloat3(inputLabel, source3))
    {
        restrictRange(min_v, max_v, source3[0]);
        restrictRange(min_v, max_v, source3[1]);
        restrictRange(min_v, max_v, source3[2]);

        source.x = source3[0];
        source.y = source3[1];
        source.z = source3[2];
        trigger = true;
    }

    if (ImGui::SliderFloat3(sliderLabel, source3, min_v, max_v))
    {
        source.x = source3[0];
        source.y = source3[1];
        source.z = source3[2];
        trigger = true;
    }
}

void ConfigureImgui::updateTransform()
{
    inputAndSlider(LIST_OF_OBJS[current_item_index].transform, -10.f, 10.f, "Transform", "Transform Input", "Transform Slider", updateObjTransform);
}

void ConfigureImgui::updateMaterialDetail()
{

    if (current_item_material == ALL_MATERIALS[0])
    {
        inputAndSlider(LIST_OF_OBJS[current_item_index].material.lambertian.albedo, 0.f, 1.f, "Albedo", "Lambertian Albedo Input", "Lambertian Albedo Slider", updateObjMaterials);
    }
    else if (current_item_material == ALL_MATERIALS[1])
    {
        inputAndSlider(LIST_OF_OBJS[current_item_index].material.metal.albedo, 0.f, 5.f, "Albedo", "Metal Albedo Input", "Metal Albedo Slider", updateObjMaterials);

        float fuzz = LIST_OF_OBJS[current_item_index].material.metal.fuzz;
        ImGui::Text("Fuzz");
        if (ImGui::SliderFloat("Metal Fuzz", &fuzz, 0.0f, 1.0f))
        {
            LIST_OF_OBJS[current_item_index].material.metal.fuzz = fuzz;
            updateObjMaterials = true;
        }
    }
    else if (current_item_material == ALL_MATERIALS[2])
    {
        float ref_idx = LIST_OF_OBJS[current_item_index].material.dielectric.ref_idx;
        ImGui::Text("Ref Index");
        if (ImGui::SliderFloat("Dielectric Ref Index", &ref_idx, 0.0f, 1.0f))
        {
            LIST_OF_OBJS[current_item_index].material.dielectric.ref_idx = ref_idx;
            updateObjMaterials = true;
        }
    }
}

void ConfigureImgui::updateMaterial()
{
    current_item_material = LIST_OF_OBJS[current_item_index].material.type;
    ImGui::Text("Loaded "); ImGui::SameLine();
    if (ImGui::BeginCombo(" Material", current_item_material)) // The second parameter is the label previewed before opening the combo.
    {
        for (int n = 0; n < ALL_MATERIALS.size(); n++)
        {
            const char* material = ALL_MATERIALS[n].c_str();
            bool is_selected = (current_item_material == material); // You can store your selection however you want, outside or inside your objects
            if (ImGui::Selectable(material, is_selected))
            {
                current_item_material = material;
                LIST_OF_OBJS[current_item_index].material.type = material;
                updateObjMaterials = true;
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
            }
        }
        ImGui::EndCombo();
    }
    updateMaterialDetail();
}

void ConfigureImgui::renderObjCP()
{
    if (showObjControlPanel)
    {
        ImGui::Begin("Obj Control Panel", &showObjControlPanel);

        if (ImGui::Button("Add New Obj"))
        {
            addNewObj = !addNewObj;
        }
        addObj();
        
        ImGui::Text("Loaded "); ImGui::SameLine();
        if (ImGui::BeginCombo(" Obj", current_item)) // The second parameter is the label previewed before opening the combo.
        {
            for (int n = 0; n < LIST_OF_OBJS.size(); n++)
            {
                const char* item = LIST_OF_OBJS[n].name.c_str();
                bool is_selected = (current_item == item); // You can store your selection however you want, outside or inside your objects
                if (LIST_OF_OBJS[n].choosed == false)
                {
                    continue;
                }
                if (ImGui::Selectable(item, is_selected))
                {
                    current_item = item;
                    current_item_index = n;
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
                }
            }
            ImGui::EndCombo();
        }
        if (current_item != NULL) {
            updateTransform();
            updateMaterial();
        }
        ImGui::End();
    }
}

void ConfigureImgui::updateLight()
{
    if (current_light.type == ALL_LIGHTS[0].c_str())
    {
        inputAndSlider(LIST_OF_AMBIENT_LIGHTS[current_light.index].intensity, 0.f, 5.f, "Intensity", "Ambient Intensity Input", "Ambient Intensity Slider", updateLights);
    }
    else if (current_light.type == ALL_LIGHTS[1].c_str())
    {
        inputAndSlider(LIST_OF_DIRECTIONAL_LIGHTS[current_light.index].intensity, 0.f, 100.f, "Intensity", "Directional Intensity Input", "Directional Intensity Slider", updateLights);

        inputAndSlider(LIST_OF_DIRECTIONAL_LIGHTS[current_light.index].direction, -10.f, 10.f, "Direction", "Directional Direction Input", "Directional Direction Slider", updateLights);
    }
}

void ConfigureImgui::addLight()
{
    if (addNewLight)
    {
        ImGui::Begin("Add Light", &addNewLight);
        ImGui::Text("Selected %d Lights", SELECTED_LIGHTS);
        for (int i = 0; i < LIST_OF_AMBIENT_LIGHTS.size(); i++)
        {
            const char* selectedLight = LIST_OF_AMBIENT_LIGHTS[i].name;
            if (ImGui::Checkbox(selectedLight, &LIST_OF_AMBIENT_LIGHTS[i].choosed))
            {
                updateLights = true;
                if (LIST_OF_AMBIENT_LIGHTS[i].choosed == false) {
                    if (current_light.name == selectedLight)
                    {
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
        for (int i = 0; i < LIST_OF_DIRECTIONAL_LIGHTS.size(); i++)
        {
            const char* selectedLight = LIST_OF_DIRECTIONAL_LIGHTS[i].name;
            if (ImGui::Checkbox(selectedLight, &LIST_OF_DIRECTIONAL_LIGHTS[i].choosed))
            {
                updateLights = true;
                if (LIST_OF_DIRECTIONAL_LIGHTS[i].choosed == false) {
                    if (current_light.name == selectedLight)
                    {
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

void ConfigureImgui::renderLightCP()
{
    if (showLightControlPanel)
    {
        ImGui::Begin("Light Control Panel", &showLightControlPanel);

        if (ImGui::Button("Add New Light"))
        {
            addNewLight = !addNewLight;
        }
        addLight();
        
        ImGui::Text("Loaded "); ImGui::SameLine();
        if (ImGui::BeginCombo(" Light", current_light.name)) // The second parameter is the label previewed before opening the combo.
        {
            for (int n = 0; n < LIST_OF_AMBIENT_LIGHTS.size(); n++)
            {
                const char* light = LIST_OF_AMBIENT_LIGHTS[n].name;
                bool is_selected = (current_light.name == light); // You can store your selection however you want, outside or inside your objects
                if (LIST_OF_AMBIENT_LIGHTS[n].choosed == false)
                {
                    continue;
                }
                if (ImGui::Selectable(light, is_selected))
                {
                    current_light.name = light;
                    current_light.index = n;
                    current_light.type = ALL_LIGHTS[0].c_str();
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
                }
            }
            for (int n = 0; n < LIST_OF_DIRECTIONAL_LIGHTS.size(); n++)
            {
                const char* light = LIST_OF_DIRECTIONAL_LIGHTS[n].name;
                bool is_selected = (current_light.name == light); // You can store your selection however you want, outside or inside your objects
                if (LIST_OF_DIRECTIONAL_LIGHTS[n].choosed == false)
                {
                    continue;
                }
                if (ImGui::Selectable(light, is_selected))
                {
                    current_light.name = light;
                    current_light.index = n;
                    current_light.type = ALL_LIGHTS[1].c_str();
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
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

void ConfigureImgui::render()
{
    // render your GUI
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSizeConstraints(ImVec2(300, 300), ImVec2(FLT_MAX, FLT_MAX));
    if (showImgui) {
        ImGui::Begin("Control Panel", &showImgui);
        ImGui::Text("io.WantCaptureMouse: %d", ImGui::GetIO().WantCaptureMouse);

        if (ImGui::Button("Obj Control Panel"))
        {
            showObjControlPanel = !showObjControlPanel;
        }

        if (ImGui::Button("Light Control Panel"))
        {
            showLightControlPanel = !showLightControlPanel;
        }
        
        renderObjCP();
        renderLightCP();
    }
    
    ImGui::End();
        
    // Render dear imgui into screen
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}