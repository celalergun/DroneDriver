#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include "../libs/stb/stb_image.h"

#include "proxy.h"
#include "reliable_sender.h"

using std::cout;
using std::endl;

struct MapCoordinates {
    float topLeftX = 59.430927f;
    float topLeftY = 24.789792f;
    float bottomRightX = 59.417272f;
    float bottomRightY = 24.836687f;
};

// 59.430927,24.789792
// 59.417272,24.836687


// Telemetry data structure
struct TelemetryData {
    float lat = 0.0f;
    float lon = 0.0f;
    float altitude = 100.0f;
    float speed = 0.0f;
    float heading = 0.0f;
    std::string mode = "MODE_GUIDED_ARMED";
    float packetLoss = 0.0f;
};

// Thread-safe shared state between proxy callback thread and GUI thread
struct SharedState {
    std::mutex mutex;
    TelemetryData telemetry;
    bool connected = false;
    std::chrono::steady_clock::time_point lastHeartbeat;
};

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Load texture from file
bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
    int image_width = 0;
    int image_height = 0;
    int channels = 0;
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, &channels, 4);
    if (image_data == nullptr) {
        fprintf(stderr, "Failed to load image: %s\n", filename);
        return false;
    }

    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;

    return true;
}

int main() {
    std::cout << "GCS UI Starting..." << std::endl;

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

#if defined(IMGUI_IMPL_OPENGL_ES2)
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(IMGUI_IMPL_OPENGL_ES3)
    const char* glsl_version = "#version 300 es";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    GLFWwindow* window = glfwCreateWindow((int)(1280 * main_scale), (int)(800 * main_scale), "GCS UI", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    // Setup dark theme
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load map texture
    GLuint mapTexture = 0;
    int mapWidth = 0, mapHeight = 0;
    bool mapLoaded = LoadTextureFromFile("ülemiste.jpg", &mapTexture, &mapWidth, &mapHeight);

    // ===== Start Proxy =====
    Proxy proxy;

    // Shared state for thread-safe telemetry updates
    SharedState shared;

    // Reliable command sender: retries until ACK
    ReliableSender reliableSender([&proxy](const mavlink_message_t& msg) {
        proxy.sendToClient(msg);
    });

    // Register callback: proxy delivers parsed MAVLink messages from drone
    proxy.onMessageFromClient([&shared, &reliableSender](const mavlink_message_t& msg) {
        // Notify reliable sender of potential ACKs
        reliableSender.onAckReceived(msg);

        std::lock_guard lock(shared.mutex);
        // cout << "Proxy callback: Received MAVLink message with ID " << msg.msgid << endl;
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                // cout << "Proxy callback: Decoding HEARTBEAT message" << endl;
                shared.lastHeartbeat = std::chrono::steady_clock::now();
                shared.connected = true;
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                // Decode MAV_MODE to string
                if (hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED)
                    shared.telemetry.mode = "AUTO_ARMED";
                else if (hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) {
                    if (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                        shared.telemetry.mode = "GUIDED_ARMED";
                    else
                        shared.telemetry.mode = "GUIDED_DISARMED";
                }
                break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                // cout << "Proxy callback: Decoding GLOBAL_POSITION_INT message" << endl;
                mavlink_global_position_int_t gpos;
                mavlink_msg_global_position_int_decode(&msg, &gpos);
                shared.telemetry.lat = gpos.lat / 1e7f;               // degE7 → deg
                shared.telemetry.lon = gpos.lon / 1e7f;               // degE7 → deg
                shared.telemetry.altitude = gpos.alt / 1000.0f;        // mm → m
                shared.telemetry.speed = gpos.vx / 100.0f;             // cm/s → m/s
                shared.telemetry.heading = gpos.hdg / 100.0f;          // cdeg → deg
                break;
            }
            default:
                break;
        }
    });
    cout << "Starting proxy..." << std::endl;
    proxy.start();

    MapCoordinates mapCoords;

    // Control state
    int packetDropPercent = 0;
    
    ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.00f);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ===== Copy shared state (thread-safe snapshot for this frame) =====
        TelemetryData telemetry;
        bool droneConnected = false;
        bool qualityGood = false;
        {
            std::lock_guard lock(shared.mutex);
            telemetry = shared.telemetry;
            droneConnected = shared.connected;
            // Check heartbeat timeout (3 seconds)
            auto elapsed = std::chrono::steady_clock::now() - shared.lastHeartbeat;
            if (elapsed > std::chrono::seconds(3)) {
                shared.connected = false;
                droneConnected = false;
            }
            qualityGood = droneConnected && (packetDropPercent < 50);
            // Update packet loss display from proxy stats
            auto total = proxy.getPacketsForwarded() + proxy.getPacketsDropped();
            if (total > 0)
                telemetry.packetLoss = 100.0f * proxy.getPacketsDropped() / total;
        }

        // Sync slider → proxy
        proxy.setPacketDropPercent(packetDropPercent);

        // Get window dimensions
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        
        // Calculate panel sizes
        float rightPanelWidth = 250.0f;
        float mapPanelWidth = display_w - rightPanelWidth - 20;
        float mapPanelHeight = display_h - 20;

        // ============ MAP PANEL ============
        ImGui::SetNextWindowPos(ImVec2(10, 10));
        ImGui::SetNextWindowSize(ImVec2(mapPanelWidth, mapPanelHeight));
        ImGui::Begin("Map", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        
        if (mapLoaded) {
            ImVec2 mapSize = ImGui::GetContentRegionAvail();
            ImVec2 cursorPos = ImGui::GetCursorScreenPos();
            
            // Calculate aspect ratio preserving size
            float imgAspect = (float)mapWidth / (float)mapHeight;
            float availAspect = mapSize.x / mapSize.y;
            
            ImVec2 displaySize;
            if (imgAspect > availAspect) {
                displaySize.x = mapSize.x;
                displaySize.y = mapSize.x / imgAspect;
            } else {
                displaySize.y = mapSize.y;
                displaySize.x = mapSize.y * imgAspect;
            }
            
            // Draw map image
            ImGui::Image((ImTextureID)(intptr_t)mapTexture, displaySize);
            
            // Handle map clicks — send target coordinates to drone
            if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
                ImVec2 mousePos = ImGui::GetMousePos();
                float relX = (mousePos.x - cursorPos.x) / displaySize.x;
                float relY = (mousePos.y - cursorPos.y) / displaySize.y;
                
                if (relX >= 0 && relX <= 1 && relY >= 0 && relY <= 1) {
                    // Convert relative position to lat/lon
                    float lat = mapCoords.topLeftX - relY * (mapCoords.topLeftX - mapCoords.bottomRightX);
                    float lon = mapCoords.topLeftY + relX * (mapCoords.bottomRightY - mapCoords.topLeftY);

                    // Send with retry-until-ACK
                    reliableSender.sendPositionTarget(
                        static_cast<int32_t>(lat * 1e7),
                        static_cast<int32_t>(lon * 1e7),
                        100.0f
                    );
                    std::cout << "[GCS] Target sent: lat=" << lat << " lon=" << lon << std::endl;
                }
            }
            
            // Draw drone position on map
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            float droneRelX = (telemetry.lon - mapCoords.topLeftY) / (mapCoords.bottomRightY - mapCoords.topLeftY);
            float droneRelY = (mapCoords.topLeftX - telemetry.lat) / (mapCoords.topLeftX - mapCoords.bottomRightX);
            float droneScreenX = cursorPos.x + droneRelX * displaySize.x;
            float droneScreenY = cursorPos.y + droneRelY * displaySize.y;
            draw_list->AddCircleFilled(ImVec2(droneScreenX, droneScreenY), 5.0f, IM_COL32(255, 50, 50, 255));
        } else {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "Failed to load ülemiste.jpg");
            ImGui::Text("Please ensure the image is in the working directory.");
        }
        
        ImGui::End();

        // ============ RIGHT PANEL ============
        float rightPanelX = display_w - rightPanelWidth - 10;
        float currentY = 10;
        float panelSpacing = 10;

        // ============ CONTROLS PANEL ============
        ImGui::SetNextWindowPos(ImVec2(rightPanelX, currentY));
        ImGui::SetNextWindowSize(ImVec2(rightPanelWidth, 100));
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        
        ImGui::SliderInt("##packetdrop", &packetDropPercent, 0, 100);
        ImGui::SameLine();
        ImGui::Text("Packet Drop %%");
        
        ImGui::Spacing();
        
        if (ImGui::Button("Hold", ImVec2(100, 30))) {
            // Send with retry-until-ACK
            reliableSender.sendHoldCommand();
            std::cout << "[GCS] HOLD command sent" << std::endl;
        }
        
        ImGui::End();
        currentY += 100 + panelSpacing;

        // ============ TELEMETRY PANEL ============
        ImGui::SetNextWindowPos(ImVec2(rightPanelX, currentY));
        ImGui::SetNextWindowSize(ImVec2(rightPanelWidth, 180));
        ImGui::Begin("Telemetry", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        
        ImGui::Text("Lat: %.6f", telemetry.lat);
        ImGui::Text("Lon: %.6f", telemetry.lon);
        ImGui::Text("Altitude: %.2f", telemetry.altitude);
        ImGui::Text("Speed: %.2f", telemetry.speed);
        ImGui::Text("Heading: %.2f", telemetry.heading);
        ImGui::Text("Mode: %s", telemetry.mode.c_str());
        ImGui::Text("Packet loss: %.2f", telemetry.packetLoss);
        
        ImGui::End();
        currentY += 180 + panelSpacing;

        // ============ CONNECTION PANEL ============
        ImGui::SetNextWindowPos(ImVec2(rightPanelX, currentY));
        ImGui::SetNextWindowSize(ImVec2(rightPanelWidth, 90));
        ImGui::Begin("Connection", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        
        // Connected status indicator
        ImVec4 connectedColor = droneConnected ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1);
        ImGui::PushStyleColor(ImGuiCol_Button, connectedColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, connectedColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, connectedColor);
        ImGui::Button("##connected", ImVec2(20, 20));
        ImGui::PopStyleColor(3);
        ImGui::SameLine();
        ImGui::Text("Connected");
        
        // Quality indicator
        ImVec4 qualityColor = qualityGood ? ImVec4(0, 1, 0, 1) : ImVec4(1, 1, 0, 1);
        ImGui::PushStyleColor(ImGuiCol_Button, qualityColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, qualityColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, qualityColor);
        ImGui::Button("##quality", ImVec2(20, 20));
        ImGui::PopStyleColor(3);
        ImGui::SameLine();
        ImGui::Text("Quality");
        
        ImGui::End();
        currentY += 90 + panelSpacing;



        // Rendering
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    reliableSender.waitForCompletion();
    proxy.stop();

    if (mapLoaded) {
        glDeleteTextures(1, &mapTexture);
    }
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
