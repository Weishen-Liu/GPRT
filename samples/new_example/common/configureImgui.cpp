#ifndef INCLUDE_VIEWER_H
#define INCLUDE_VIEWER_H
#include "./viewer.hpp"
#endif

#ifndef INCLUDE_CONTANTS
#define INCLUDE_CONTANTS
#include "./constants.hpp"
#endif

#include <cstring>
#include <string>

struct ConfigureImgui {
    bool showImgui = true;
    bool showLightControlPanel = false;
    bool showObjControlPanel = false;
    bool addNewObj = false;
    bool updateSelectedObj = false;
    bool objCountWarning = false;

    void render();
    void renderObjCP();
    void renderLightCP();

    void initObj();
    void addObj();
    void showSelectedObjCountWarning();
    void updateTransform();
    void updateMaterial();
};

void ConfigureImgui::initObj()
{
    std::cout<<"Init Obj"<<std::endl;
    for (int i = 0; i < ALL_MODEL_PATH.size(); i++)
    {
        Obj newObj;
        newObj.name = ALL_MODEL_NAME[i];
        newObj.path = ALL_MODEL_PATH[i];
        newObj.transform = INITIAL_TRANSFORM[i];
        LIST_OF_OBJS.push_back(newObj);
    }
    // Need to at least render one obj
    LIST_OF_OBJS[0].choosed = true;
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
                updateSelectedObj = true;
                if (LIST_OF_OBJS[i].choosed == false) {
                    if (current_item == item)
                    {
                        current_item = NULL;
                        current_item_index = -1;
                    }
                    if (SELECTED_OBJS == 1) {
                        // Need to at least render one objs
                        objCountWarning = true;
                        LIST_OF_OBJS[i].choosed = true;
                        break;
                    } else {
                        SELECTED_OBJS -= 1;
                    }
                } else {
                    SELECTED_OBJS += 1;
                    objCountWarning = false;
                }
            }
        }
        ImGui::End();

        showSelectedObjCountWarning();
    }
}

void ConfigureImgui::showSelectedObjCountWarning()
{
    if (objCountWarning)
    {
        ImGui::Begin("Warning", &objCountWarning);
        ImGui::Text("Cannot uncheck this one.\nNeed to render at least one obj");
        ImGui::End();
    }
}

void restrictRange(float min_range, float max_range, float &input)
{
    if (input > max_range) input = max_range;
    if (input < min_range) input = min_range;
}

void ConfigureImgui::updateTransform()
{
    float transform3[3] = {
        LIST_OF_OBJS[current_item_index].transform.x,
        LIST_OF_OBJS[current_item_index].transform.y,
        LIST_OF_OBJS[current_item_index].transform.z
    };
    ImGui::Text("Transform");
    if (ImGui::InputFloat3("Input", transform3))
    {
        restrictRange(-10.f, 10.f, transform3[0]);
        restrictRange(-10.f, 10.f, transform3[1]);
        restrictRange(-10.f, 10.f, transform3[2]);

        LIST_OF_OBJS[current_item_index].transform.x = transform3[0];
        LIST_OF_OBJS[current_item_index].transform.y = transform3[1];
        LIST_OF_OBJS[current_item_index].transform.z = transform3[2];
        updateSelectedObj = true;
    }

    if (ImGui::SliderFloat3("Slider", transform3, -10.f, 10.f))
    {
        LIST_OF_OBJS[current_item_index].transform.x = transform3[0];
        LIST_OF_OBJS[current_item_index].transform.y = transform3[1];
        LIST_OF_OBJS[current_item_index].transform.z = transform3[2];
        updateSelectedObj = true;
    }
}

void ConfigureImgui::updateMaterial()
{

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
        
        ImGui::Text("Loaded Obj"); ImGui::SameLine();
        if (ImGui::BeginCombo("##combo", current_item)) // The second parameter is the label previewed before opening the combo.
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

void ConfigureImgui::renderLightCP()
{
    if (showLightControlPanel)
    {
        ImGui::Begin("Light Control Panel", &showLightControlPanel);
        ImGui::SliderFloat("float", &imgui_test_input, 0.0f, 1.0f);
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