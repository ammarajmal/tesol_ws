#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/SetCameraInfo.h>

class CameraNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    image_transport::ImageTransport it_;
    std::string camera_name_;
    std::string device_id_;
    std::string camera_frame_id_;
    int image_width_;
    int image_height_;
    int frame_rate_;
    std::string calibration_file_;
    image_transport::Publisher pub_image_raw_;
    ros::Publisher pub_camera_info_;
    ros::ServiceServer srv_set_camera_info_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
    cv::VideoCapture cap_;

public:
    CameraNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), it_(nh) {
        try {
            InitParams();
            InitCamera();
            InitCameraInfoManager();
            AdvertiseTopics();
            // AdvertiseService();
        } catch (const std::exception& e) {
            ROS_ERROR("Initialization failed: %s", e.what());
            ros::shutdown();
        }
    }

    void InitParams() {
        nh_private_.param("camera_name", camera_name_, std::string("usb_cam"));
        nh_private_.param("device_id", device_id_, std::string("/dev/video0"));
        nh_private_.param("camera_frame_id", camera_frame_id_, std::string("usb_cam"));
        nh_private_.param("image_width", image_width_, 1920);
        nh_private_.param("image_height", image_height_, 1080);
        nh_private_.param("frame_rate", frame_rate_, 60);
        nh_private_.param("calibration_file", calibration_file_, std::string(""));
    }

    void InitCamera() {
        if (!cap_.open(device_id_, cv::CAP_V4L2)) {
            throw std::runtime_error("Could not open video device: " + device_id_);
        }
        //             // Attempt to set the camera to use MJPG by specifying the FourCC code
        // // Note: The actual effectiveness of this command can vary by camera and driver support
        // bool isSetMJPG = cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // bool result_width = cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        // bool result_height = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
        // if (!result_width || !result_height) {
        //     ROS_WARN("Failed to set camera resolution to %dx%d.", image_width_, image_height_);
        // }
    // if (!isSetMJPG) {
    //     ROS_WARN("Unable to set camera to MJPG format; defaulting to camera's current setting.");
    // }
    }

    void InitCameraInfoManager() {
        std::string camera_info_url;
        nh_private_.param("camera_info_url", camera_info_url, std::string(""));
        cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(nh_private_, camera_name_, camera_info_url);

        if (!calibration_file_.empty() && !cam_info_manager_->validateURL(calibration_file_)) {
            ROS_WARN("Calibration file is not valid. Calibration file will be ignored.");
        } else if (!calibration_file_.empty()) {
            cam_info_manager_->loadCameraInfo(calibration_file_);
        }
    }

void AdvertiseTopics() {
    // Fetch the resolved name of the node, which includes the namespace
    std::string node_name = ros::this_node::getName();

    // Use the node name to prefix the topics
    pub_image_raw_ = it_.advertise(node_name + "/image_raw", 1);
    pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>(node_name + "/camera_info", 1);
}


    void AdvertiseService() {
        ROS_INFO("AdvertiseService started.. ");
        std::string node_name = ros::this_node::getName();
        srv_set_camera_info_ = nh_.advertiseService("set_camera_info", &CameraNode::SetCameraInfoService, this);
    }

    bool SetCameraInfoService(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res) {
        try {
            if (cam_info_manager_->setCameraInfo(req.camera_info)) {
                res.success = true;
                res.status_message = "Camera info set successfully.";
                ROS_INFO("Camera info set successfully.");
            } else {
                res.success = false;
                res.status_message = "Failed to set camera info.";
                ROS_ERROR("Failed to set camera info.");
            }
            return true;
            ROS_INFO("SetCameraInfo service finished.. ");
        } catch (const std::exception& e) {
            ROS_ERROR("SetCameraInfo service failed: %s", e.what());
            res.success = false;
            res.status_message = std::string("Service failure: ") + e.what();
            return false;
        }
    }

    void PublishImage() {
        cv::Mat frame;
        if (cap_.read(frame)) { // Capture a frame
            try {
                cv::resize(frame, frame, cv::Size(image_width_, image_height_), 0, 0, cv::INTER_LINEAR);

                ros::Time current_time = ros::Time::now();

                cv_bridge::CvImage cv_image;
                cv_image.image = frame;
                cv_image.encoding = sensor_msgs::image_encodings::BGR8;
                sensor_msgs::Image ros_image;
                cv_image.toImageMsg(ros_image);
                ros_image.header.frame_id = camera_frame_id_;
                ros_image.header.stamp = current_time; // Use the time at which the image was captured
                pub_image_raw_.publish(ros_image);

                sensor_msgs::CameraInfo camera_info = cam_info_manager_->getCameraInfo();
                camera_info.header.frame_id = camera_frame_id_;
                camera_info.header.stamp = current_time; // Use the time at which the image was captured
                camera_info.width = image_width_;
                camera_info.height = image_height_;
                pub_camera_info_.publish(camera_info);
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to publish image: %s", e.what());
            }
        }
    }

    void Spin() {
        ros::Rate loop_rate(frame_rate_);
        while (ros::ok()) {
            PublishImage();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    try {
        CameraNode camera_node(nh, nh_private);
        camera_node.Spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Camera node initialization failed: %s", e.what());
        return 1;
    }
    return 0;
}
