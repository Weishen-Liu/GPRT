#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

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
        newObj.material.type = ALL_MATERIALS[0].c_str();
        newObj.material.lambertian.albedo = lambertian_color;
        newObj.material.metal.albedo = metal_color;
        newObj.material.metal.fuzz = metal_fuzz;
        newObj.material.dielectric.ref_idx = dielectric_ref_idx;

        LIST_OF_OBJS.push_back(newObj);
    }

    for (auto &each_obj: LIST_OF_OBJS) {
        for (auto &each_instance: each_obj.instances) {
            if (each_instance.choosed) {
                each_obj.SELECTED_OBJ_INSTANCE++;
            }
        }
    }
}

void ConfigureImgui::initLight()
{
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

    for (auto &each_light: LIST_OF_AMBIENT_LIGHTS) {
        if (each_light.choosed) {
            SELECTED_LIGHTS++;
        }
    }
    for (auto &each_light: LIST_OF_DIRECTIONAL_LIGHTS) {
        if (each_light.choosed) {
            SELECTED_LIGHTS++;
        }
    }
}

void ConfigureImgui::createDefaultInstance(Obj& obj)
{
    for (int i = 0; i < obj.generateInstance; i++)
    {
        Obj::Instance instance;
        instance.transform = obj.defaultTransform;
        instance.name = obj.name + " " +std::to_string(obj.instanceUniqueName);
        obj.instanceUniqueName++;
        obj.instances.push_back(instance);
    }
    updateObjSelection = true;
}

void ConfigureImgui::addObjInstance(Obj& obj)
{
    if (obj.addObjInstanceWindow)
    {
        const char* item = obj.name.c_str();
        std::string windowName = "Instances for " + obj.name;
        ImGui::Begin(windowName.c_str(), &obj.addObjInstanceWindow);
        ImGui::Text("Generate"); ImGui::SameLine();
        std::string generateInstanceName = " Instance for " + obj.name;
        ImGui::SliderInt(generateInstanceName.c_str(), &obj.generateInstance, 1, 3);
        if (ImGui::Button("Create Instances"))
        {
            createDefaultInstance(obj);
        }
        ImGui::Text("");
        ImGui::Text("Selected %d Instances under Obj '%s'", obj.SELECTED_OBJ_INSTANCE, item);
        for (int eachInstance = 0; eachInstance < obj.instances.size(); eachInstance++)
        {
            const char* instance_name = obj.instances[eachInstance].name.c_str();
            if (ImGui::Checkbox(instance_name, &obj.instances[eachInstance].choosed))
            {
                updateObjSelection = true;
                if (obj.instances[eachInstance].choosed == false) {
                    if (obj.current_instance == instance_name)
                    {
                        obj.current_instance = NULL;
                        obj.current_instance_index = -1;
                    }
                    obj.SELECTED_OBJ_INSTANCE -= 1;
                } else {
                    obj.SELECTED_OBJ_INSTANCE += 1;
                }
            }
        }
        ImGui::End();
    }
}

void ConfigureImgui::objInstances()
{
    for (int eachObj = 0; eachObj < LIST_OF_OBJS.size(); eachObj++)
    {
        const char* item = LIST_OF_OBJS[eachObj].name.c_str();
        if (LIST_OF_OBJS[eachObj].openInstanceWindow)
        {
            ImGui::Begin(item, &LIST_OF_OBJS[eachObj].openInstanceWindow);
            // Add Instance Window
            if (ImGui::Button("Add Instance"))
            {
                LIST_OF_OBJS[eachObj].addObjInstanceWindow = !LIST_OF_OBJS[eachObj].addObjInstanceWindow;
            }
            addObjInstance(LIST_OF_OBJS[eachObj]);

            // Configure Instance Transform
            ImGui::Text("Loaded "); ImGui::SameLine();
            if (ImGui::BeginCombo(" Instance", LIST_OF_OBJS[eachObj].current_instance)) // The second parameter is the label previewed before opening the combo.
            {
                for (int n = 0; n < LIST_OF_OBJS[eachObj].instances.size(); n++)
                {
                    const char* isntance_name = LIST_OF_OBJS[eachObj].instances[n].name.c_str();
                    bool is_selected = (LIST_OF_OBJS[eachObj].current_instance == isntance_name); // You can store your selection however you want, outside or inside your objects
                    if (LIST_OF_OBJS[eachObj].instances[n].choosed == false)
                    {
                        continue;
                    }
                    if (ImGui::Selectable(isntance_name, is_selected))
                    {
                        LIST_OF_OBJS[eachObj].current_instance = isntance_name;
                        LIST_OF_OBJS[eachObj].current_instance_index = n;
                    }
                    if (is_selected)
                    {
                        ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
                    }
                }
                ImGui::EndCombo();
            }
            if (LIST_OF_OBJS[eachObj].current_instance != NULL) {
                updateTransform(LIST_OF_OBJS[eachObj]);
                updateMaterial(LIST_OF_OBJS[eachObj]);
            }
            ImGui::End();
        }
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

void ConfigureImgui::updateTransform(Obj& obj)
{
    inputAndSlider(obj.instances[obj.current_instance_index].transform, -10.f, 10.f, "Transform", "Transform Input", "Transform Slider", updateObjTransform);
}

void ConfigureImgui::updateMaterialDetail(Obj& obj)
{

    if (obj.current_instance_material == ALL_MATERIALS[0])
    {
        inputAndSlider(obj.material.lambertian.albedo, 0.f, 1.f, "Albedo", "Lambertian Albedo Input", "Lambertian Albedo Slider", updateObjMaterials);
    }
    else if (obj.current_instance_material == ALL_MATERIALS[1])
    {
        inputAndSlider(obj.material.metal.albedo, 0.f, 5.f, "Albedo", "Metal Albedo Input", "Metal Albedo Slider", updateObjMaterials);

        float fuzz = obj.material.metal.fuzz;
        ImGui::Text("Fuzz");
        if (ImGui::SliderFloat("Metal Fuzz", &fuzz, 0.0f, 1.0f))
        {
            obj.material.metal.fuzz = fuzz;
            updateObjMaterials = true;
        }
    }
    else if (obj.current_instance_material == ALL_MATERIALS[2])
    {
        float ref_idx = obj.material.dielectric.ref_idx;
        ImGui::Text("Ref Index");
        if (ImGui::SliderFloat("Dielectric Ref Index", &ref_idx, 0.0f, 1.0f))
        {
            obj.material.dielectric.ref_idx = ref_idx;
            updateObjMaterials = true;
        }
    }
}

void ConfigureImgui::updateMaterial(Obj& obj)
{
    obj.current_instance_material = obj.material.type;
    ImGui::Text("Loaded "); ImGui::SameLine();
    if (ImGui::BeginCombo(" Material", obj.current_instance_material)) // The second parameter is the label previewed before opening the combo.
    {
        for (int n = 0; n < ALL_MATERIALS.size(); n++)
        {
            const char* material = ALL_MATERIALS[n].c_str();
            bool is_selected = (obj.current_instance_material == material); // You can store your selection however you want, outside or inside your objects
            if (ImGui::Selectable(material, is_selected))
            {
                obj.current_instance_material = material;
                obj.material.type = material;
                updateObjMaterials = true;
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
            }
        }
        ImGui::EndCombo();
    }
    updateMaterialDetail(obj);
}

void ConfigureImgui::renderObjCP()
{
    if (showObjControlPanel)
    {
        ImGui::Begin("Obj Control Panel", &showObjControlPanel);

        for (int i = 0; i < LIST_OF_OBJS.size(); i++)
        {
            const char* item = LIST_OF_OBJS[i].name.c_str();
            if (ImGui::Button(item))
            {
                LIST_OF_OBJS[i].openInstanceWindow = !LIST_OF_OBJS[i].openInstanceWindow;
            }
        }
        objInstances();
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
        if (ImGui::Button("Add New Ambient Light"))
        {
            AmbientLight sampleAmbientLight;
            sampleAmbientLight.name = "Ambient Light " + std::to_string(ambientLightUniqueName);
            ambientLightUniqueName++;
            sampleAmbientLight.intensity = float3(1.f, 1.f, 1.f);
            sampleAmbientLight.choosed = false;
            LIST_OF_AMBIENT_LIGHTS.push_back(sampleAmbientLight);
        }

        if (ImGui::Button("Add New Directional Light"))
        {
            DirectionalLight sampleDirectionalLight;
            sampleDirectionalLight.name = "Directional Light " + std::to_string(directionalLightUniqueName);
            directionalLightUniqueName++;
            sampleDirectionalLight.intensity = float3(1.f, 1.f, 1.f);
            sampleDirectionalLight.direction = float3(0.f, -5.f, 0.f);
            sampleDirectionalLight.choosed = false;
            LIST_OF_DIRECTIONAL_LIGHTS.push_back(sampleDirectionalLight);
        }

        ImGui::Text("Selected %d Lights", SELECTED_LIGHTS);
        for (int i = 0; i < LIST_OF_AMBIENT_LIGHTS.size(); i++)
        {
            const char* selectedLight = LIST_OF_AMBIENT_LIGHTS[i].name.c_str();
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
            const char* selectedLight = LIST_OF_DIRECTIONAL_LIGHTS[i].name.c_str();
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
                const char* light = LIST_OF_AMBIENT_LIGHTS[n].name.c_str();
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
                const char* light = LIST_OF_DIRECTIONAL_LIGHTS[n].name.c_str();
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