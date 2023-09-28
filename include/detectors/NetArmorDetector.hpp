// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
// openvino
#include "helios_autoaim_parameters.hpp"
#include"openvino/openvino.hpp"
#include"ie/inference_engine.hpp"
#include"openvino/core/core.hpp"
#include"openvino/runtime/infer_request.hpp"
// opencv
#include"opencv2/opencv.hpp"
#include"opencv2/dnn.hpp"
#include"opencv2/highgui.hpp"

#include "BaseDetector.hpp"
#include "PnPSolver.hpp"

const int NUM_CLASS = 9;//类别总数
const int NUM_COLORS = 2;//颜色
const float NMS_THRESH = 0.2;//NMS阈值
const std::vector<std::string> NUMBER_LABEL {
    "Guard",
    "1",
    "2",
    "3",
    "4",
    "5",
    "Output",
    "Base_Low",
    "Base_High"
};


namespace helios_cv {
//存储检测结果
struct Object{
    //分类
    int label;
    //颜色（这俩都同上）
    int color;
    //置信度
    float confidence;
    //左上角开始逆时针四个点
    cv::Point2f p1, p2, p3, p4;
    //外接矩形，nms非极大抑制用
    cv::Rect_<float> rect;
};

//特征图大小和相对于输入图像的步长大小
struct GridAndStride{
    int grid0;
    int grid1;
    int stride;
};

class NetArmorDetector : public BaseDetector {
public:
    NetArmorDetector(helios_autoaim::Params::Detector::ArmorDetector detector_params);

    void set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) override;

    bool init_detector(helios_autoaim::Params::Detector detector_param) override;

    helios_rs_interfaces::msg::Armors detect_targets(const cv::Mat& images) override;

    void draw_results(cv::Mat& img) override;

    void set_params(helios_autoaim::Params::Detector detector_params) override;

    helios_autoaim::Params::Detector detector_params_;
private:
    int argmax(const float* ptr, int len);
    cv::Mat static_resize(cv::Mat src);
    void generate_grids_and_stride(const int w, const int h, const int strides[], std::vector<GridAndStride> &grid_strides);
    void generate_yolox_proposal(std::vector<GridAndStride> &grid_strides, const float * output_buffer, float prob_threshold, std::vector<Object>& object, float scale);
    void qsort_descent_inplace(std::vector<Object> & faceobjects, int left, int right);
    void qsort_descent_inplace(std::vector<Object>& objects);
    void nms_sorted_bboxes(std::vector<Object> & faceobjects, std::vector<int>& picked, float nms_threshold);
    /**
     * @brief 判断装甲板的类型
     * 
     * @param object 
     * @return true 
     * @return false 
     */
    ArmorType judge_armor_type(const Object& object);
    /**
     * @brief 获取模型的输出后对结果进行解码
     * @param output_buffer 结果的首地址
     * @param object 解码后的结果保存在这里，具体看Object的定义
     * @param scale 输入图片对于原图片的缩放比例
     * 
     */
    void decode(const float* output_buffer, std::vector<Object>& object, float scale);
    // float distance(cv::Point p1, cv::Point p2);

    std::string model_path_;//模型路径
    // pnp solver
    std::shared_ptr<PnPSolver> pnp_solver_;

    // params
    helios_autoaim::Params::Detector::ArmorDetector params_;
    // camera info
    cv::Point2f cam_center_;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;

    /*----以下都是openvino的核心组件----*/
    ov::Core core_;
    ov::CompiledModel complied_model_;
    ov::InferRequest infer_request_;
    ov::Tensor input_node_;
    ov::Shape tensor_shape_;
    ov::Output<const ov::Node> input_port_;


    cv::Mat blob_;//可输入模型的数据

    float scale_;//输入大小（416*416）和原图长边的比例

};

} // helios_cv