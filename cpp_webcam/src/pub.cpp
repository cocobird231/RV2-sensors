#include <atomic>
#include "header.h"
#include "vehicle_interfaces/utils.h"

static std::atomic<bool> __globalExitFlag(false);

int main(int argc, char** argv)
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
    std::shared_ptr<const Params> params;
    {
        auto tmpNode = vehicle_interfaces::GenTmpNode<Params>("tmp_webcam_params_");
        params = std::make_shared<const Params>(tmpNode->nodeName + "_params_node");
    }

    auto pubNode = std::make_shared<RGBImagePublisher>(params);
    pubNode->addProcedureMonitor("Camera", std::chrono::nanoseconds((int64_t)(params->cameraProcTimeout_ms * 1e6)));

    auto execPubNode = vehicle_interfaces::ExecNode(pubNode);
    execPubNode.start();

    cv::VideoCapture cap;
    cap.open(params->camera_cap_id);
    if (!cap.isOpened())
    {
        std::cerr << "Unable to open camera. Exit program in 1s...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return EXIT_FAILURE;
    }
    std::cout << "Camera opened successfully\n";
    cap.set(cv::CAP_PROP_FRAME_WIDTH, params->camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, params->camera_height);
    cap.set(cv::CAP_PROP_FPS, params->camera_fps);

    cv::Size pubImgSize(params->topic_Webcam_width, params->topic_Webcam_height);
    cv::Mat frame;

    int ret = cap.read(frame);
    if (!ret || frame.rows <= 0 || frame.cols <= 0)
    {
        std::cerr << "Unable to retrieve image. Exit program in 1s...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return EXIT_FAILURE;
    }
    if (frame.size() != pubImgSize)
    std::cout << "The image will be resized into " << pubImgSize.width << "x" << pubImgSize.height 
                << ". Current image size: " << frame.cols << "x" << frame.rows << std::endl;

    while (!__globalExitFlag.load())
    {
        ret = cap.read(frame);
        if (!ret || frame.rows <= 0 || frame.cols <= 0)
        {
            std::cerr << "Unable to retrieve image\n";
            return EXIT_FAILURE;
        }
        if (!params->camera_use_color)
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        if (frame.size() != pubImgSize)
            cv::resize(frame, frame, pubImgSize);
        pubNode->pubImage(frame);
        pubNode->updateProcedureMonitor("Camera", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        cv::waitKey(1);
    }

    execPubNode.stop();
    cap.release();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
