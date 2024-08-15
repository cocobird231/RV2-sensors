#include "header.h"

#define ENABLE_TRACKING 0
#define ENABLE_SPATIAL_MAPPING 0
#define SHOW_CV_WINDOW 0
#define USE_KEYBOARD_INPUT 0

static std::atomic<bool> __globalExitFlag(false);

void ZedTh(const std::shared_ptr<const Params> params, const std::shared_ptr<ZEDNode> zedNode, int id)
{
    sl::Camera zed;
    sl::InitParameters init_parameters;
    if (params->camera_caps.size() == 0)
    {
        printf("[ZedTh] No camera selected\n");
        return;
    }
    int cameraCapId = (int)params->camera_caps[id];
    if (cameraCapId > 9999999)// SN code
    {
        init_parameters.input.setFromSerialNumber(cameraCapId);
    }
    else if (cameraCapId > 9)// GMSL
    {
        init_parameters.input.setFromCameraID(cameraCapId - 10);
    }
    else if (cameraCapId >= 0)// USB
    {
        init_parameters.input.setFromCameraID(cameraCapId);
    }
    else if (cameraCapId == -1)// SVO
    {
        init_parameters.input.setFromSVOFile(params->camera_svo_path.c_str());
    }
    else
    {
        printf("[ZedTh(%d)] Camera ID/SN not recognized\n", cameraCapId);
        return;
    }

    init_parameters.depth_mode = (sl::DEPTH_MODE)params->camera_depth_quality;
    init_parameters.coordinate_units = (sl::UNIT)params->camera_depth_unit;
    // init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    init_parameters.depth_maximum_distance = 8.0;
    printf("[ZedTh(%d)] Depth mode: %d\n", cameraCapId, params->camera_depth_quality);
    printf("[ZedTh(%d)] Depth unit: %d\n", cameraCapId, params->camera_depth_unit);

    init_parameters.camera_fps = (int)params->camera_fps;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    printf("[ZedTh(%d)] Camera FPS: %d\n", cameraCapId, zed.getInitParameters().camera_fps);

    if (returned_state != sl::ERROR_CODE::SUCCESS)// Quit if an error occurred
    {
        printf("[ZedTh(%d)] Open Camera failed: %d\n", cameraCapId, returned_state);
        zed.close();
        return;
    }

    auto camera_infos = zed.getCameraInformation();
    sl::Pose pose;
    sl::POSITIONAL_TRACKING_STATE tracking_state = sl::POSITIONAL_TRACKING_STATE::OFF;

    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = params->confidenceThreshold;

    auto resolution = camera_infos.camera_configuration.resolution;
    sl::Resolution display_resolution((int)(resolution.width * params->downsampleFactor), (int)(resolution.height * params->downsampleFactor));
    printf("[ZedTh(%d)] Resolution: %dx%d\n", cameraCapId, display_resolution.width, display_resolution.height);

    sl::Mat leftRgbSlMat(display_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    sl::Mat rightRgbSlMat(display_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    sl::Mat depthSlMat(display_resolution, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    sl::Mat pcSlMat(display_resolution, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

    zedNode->addZEDPublisher((int)params->camera_topic_ids[id], zed);
    zedNode->addProcedureMonitor("ZED_" + std::to_string(id), std::chrono::nanoseconds((int64_t)(params->zedThProcTimeout_ms * 1e6)));

    while (!__globalExitFlag.load())
    {
        // Grab a new image
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
        {
            // Retrieve the left image
            if (params->camera_use_color)
            {
                zed.retrieveImage(leftRgbSlMat, sl::VIEW::LEFT, sl::MEM::CPU, display_resolution);
                zed.retrieveImage(rightRgbSlMat, sl::VIEW::RIGHT, sl::MEM::CPU, display_resolution);
            }
            if (params->camera_use_depth)
                zed.retrieveMeasure(depthSlMat, sl::MEASURE::DEPTH, sl::MEM::CPU, display_resolution);
            if (params->camera_use_pc)
            {
                zed.retrieveMeasure(pcSlMat, sl::MEASURE::XYZBGRA, sl::MEM::GPU, display_resolution);
                tracking_state = zed.getPosition(pose);
            }
            sl::Timestamp last_image_timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);

            if (params->camera_use_color)
            {
                // zedNode->pubRGB((int)params->camera_topic_ids[id], leftRgbSlMat, last_image_timestamp);
                zedNode->pubLeftRgb((int)params->camera_topic_ids[id], leftRgbSlMat, last_image_timestamp);
                zedNode->pubRightRgb((int)params->camera_topic_ids[id], rightRgbSlMat, last_image_timestamp);
                zedNode->pubCamInfo((int)params->camera_topic_ids[id]);
            }

            if (params->camera_use_depth)
            {
                zedNode->pubDepth((int)params->camera_topic_ids[id], depthSlMat, last_image_timestamp);
            }

            if (params->camera_use_pc)
            {
                if (pcSlMat.getHeight() > 0 && pcSlMat.getWidth() > 0 && pcSlMat.updateCPUfromGPU() == sl::ERROR_CODE::SUCCESS)
                {
                    zedNode->pubPC((int)params->camera_topic_ids[id], pcSlMat, pose, last_image_timestamp);
                }
            }

            if (params->camera_use_sens)
            {
                sl::SensorsData sensData;
                if (zed.getSensorsData(sensData, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS)
                {
                    zedNode->pubImu((int)params->camera_topic_ids[id], sensData);
                    zedNode->pubEnv((int)params->camera_topic_ids[id], sensData);
                }
            }
            zedNode->updateProcedureMonitor("ZED_" + std::to_string(id), vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
        else
        {
            zedNode->updateProcedureMonitor("ZED_" + std::to_string(id), vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_WARN, "Grab failed");
        }
    }

    leftRgbSlMat.free();
    rightRgbSlMat.free();
    pcSlMat.free();
    zed.close();
    zedNode->removeZEDPublisher((int)params->camera_topic_ids[id]);
}

int main(int argc, char **argv)
{
    // Ctrl-c handler
    signal(SIGINT, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGTERM handler
    signal(SIGTERM, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGKILL handler
    signal(SIGKILL, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    rclcpp::init(argc, argv);
    std::shared_ptr<Params> params;
    {
        auto tmpNode = vehicle_interfaces::GenTmpNode<Params>("tmp_zed_params_");
        params = std::make_shared<Params>(tmpNode->nodeName + "_params_node");
    }

    std::thread params_thread(vehicle_interfaces::SpinNode, params, "params");

    if (params->camera_use_svo)
    {
        if (params->camera_topic_ids.size() < 1)
        {
            printf("[main] No camera topic ID specified for SVO playback.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return EXIT_FAILURE;
        }
        params->camera_caps = { -1.0 };
    }
    else
    {
        if (params->camera_caps.size() != params->camera_topic_ids.size() || params->camera_caps.size() == 0)
        {
            printf("[main] Number of camera IDs and topic IDs do not match or empty.\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return EXIT_FAILURE;
        }
        for (auto& i : sl::Camera::getDeviceList())
            std::cout << "[" << i.camera_state << "] " << i.camera_model << " " << i.id << " [" << i.serial_number << "](" << i.path << ")\n";
    }

    auto zedNode = std::make_shared<ZEDNode>(params);
    std::deque<vehicle_interfaces::unique_thread> thQue;

    for (int i = 0; i < params->camera_caps.size(); i++)
    {
        thQue.push_back(vehicle_interfaces::make_unique_thread(ZedTh, params, zedNode, i));
    }

    while (!__globalExitFlag.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (auto &th : thQue)
    {
        th.reset();
    }
    zedNode.reset();

    rclcpp::shutdown();

    return 0;
}
