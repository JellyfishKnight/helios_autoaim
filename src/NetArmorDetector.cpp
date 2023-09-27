// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "NetArmorDetector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <helios_rs_interfaces/msg/detail/armor__struct.hpp>

namespace helios_cv {
NetArmorDetector::NetArmorDetector(helios_autoaim::Params::Detector::ArmorDetector detector_params) {
    params_ = detector_params;
}

void NetArmorDetector::set_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr cam_info) {
    cam_info_ = cam_info;
    cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
    pnp_solver_ = std::make_shared<PnPSolver>(cam_info->k, cam_info->d);
}

bool NetArmorDetector::init_detector(helios_autoaim::Params::Detector detector_param) {
    params_ = detector_param.armor_detector;
    model_path_ = ament_index_cpp::get_package_share_directory("helios_autoaim") + 
                    "model/armor.onnx";
    complied_model_ = core_.compile_model(model_path_, "CPU");//上车后可以改成IGPU（核显）
    infer_request_ = complied_model_.create_infer_request();
    input_node_ = infer_request_.get_input_tensor();
    tensor_shape_ = input_node_.get_shape();
    input_port_ = complied_model_.input();
    return true;
}

helios_rs_interfaces::msg::Armors NetArmorDetector::detect_targets(const cv::Mat& image) {
    std::vector<helios_rs_interfaces::msg::Armor> armor_;
    //对图像进行处理，使其变成可以传给模型的数据类型
    cv::Mat pre_img = static_resize(image);
    cv::dnn::blobFromImage(pre_img, blob_, 1.0, cv::Size(cam_info_->width, cam_info_->height), cv::Scalar(), false, false);
    //把数据传给模型
    ov::Tensor input_tensor(input_port_.get_element_type(), input_port_.get_shape(), blob_.ptr(0));
    infer_request_.set_input_tensor(input_tensor);
    //执行推理
    infer_request_.infer();
    //得到推理结果
    const ov::Tensor& output = infer_request_.get_output_tensor(0);
    const float* output_buffer = output.data<const float>();
    std::vector<Object> objects;
    //对推理结果进行解码
    decode(output_buffer, objects, scale_);
    ///TODO: solve pnp

    //设置返回结果
    // for (auto &object : objects) {
    //     if (object.confidence < 0.5) {
    //         continue;
    //     }
    //     Armor armor_target;
    //     armor_target.label = object.label;
    //     armor_target.color = object.color;
    //     armor_target.p1 = object.p1;
    //     armor_target.p2 = object.p2;
    //     armor_target.p3 = object.p3;
    //     armor_target.p4 = object.p4;

    //     armor_.emplace_back(armor_target);
    //     //std::cout<<object.conf<<std::endl;
    // }
    
    return armor_;

}

void NetArmorDetector::draw_results(cv::Mat& img) {
    
}

void NetArmorDetector::set_params(helios_autoaim::Params::Detector detector_params) {
    params_ = detector_params.armor_detector;
}

int NetArmorDetector::argmax(const float* ptr, int len) {
    int arg_max=0;
    for(int i=1; i<len; i++){
        if(ptr[i]>ptr[arg_max]){
            arg_max=i;
        }
    }
    return arg_max;
}

cv::Mat NetArmorDetector::static_resize(cv::Mat src) {
    //求出模型的输入大小（416*416）相对于原图长边的比值
    scale_ = std::min(cam_info_->width / (src.cols * 1.0), cam_info_->height / (src.rows * 1.0));
    int unpad_w = scale_*src.cols;
    int unpad_h = scale_*src.rows;
    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(src, re, re.size());
    cv::Mat out(cam_info_->width, cam_info_->height, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Rect roi(0, 0, re.cols, re.rows);
    re.copyTo(out(roi));
    return out;
}

void NetArmorDetector::generate_grids_and_stride(const int w, const int h, const int strides[], 
                                                    std::vector<GridAndStride> &grid_strides) {
    for (int i = 0; i < 3; i++) {
        int num_grid_w = w / strides[i];
        int num_grid_h = h / strides[i];
        for (int g1 = 0; g1 < num_grid_h; g1++) {
            for (int g0 = 0; g0 < num_grid_w; g0++) {
                grid_strides.push_back((GridAndStride{g0, g1, strides[i]}));
            }
        }
    }
}

void NetArmorDetector::generate_yolox_proposal(std::vector<GridAndStride> &grid_strides, const float * output_buffer, 
                                                  float prob_threshold, std::vector<Object>& objects, float scale) {
    const int num_anchors = grid_strides.size();

    for (int anchor_idx = 0; anchor_idx<num_anchors; anchor_idx++) {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        const int basic_pos = anchor_idx * (9 + NUM_CLASS + NUM_COLORS);//4个点的xy坐标+1个置信度+颜色+类别

        //4个关键点
        float x1 = (output_buffer[basic_pos + 0] + grid0) * stride / scale;
        float y1 = (output_buffer[basic_pos + 1] + grid1) * stride / scale;
        float x2 = (output_buffer[basic_pos + 2] + grid0) * stride / scale;
        float y2 = (output_buffer[basic_pos + 3] + grid1) * stride / scale;
        float x3 = (output_buffer[basic_pos + 4] + grid0) * stride / scale;
        float y3 = (output_buffer[basic_pos + 5] + grid1) * stride / scale;
        float x4 = (output_buffer[basic_pos + 6] + grid0) * stride / scale;
        float y4 = (output_buffer[basic_pos + 7] + grid1) * stride / scale;
        //置信度最大颜色
        int color_idx = argmax(output_buffer + basic_pos + 9, NUM_COLORS);//获取最大的颜色置信度索引
        float color_conf = output_buffer[basic_pos + 9 + color_idx];//最大的颜色置信度的值

        //置信度最大的类别
        int class_idx = argmax(output_buffer + basic_pos + 9 + NUM_COLORS, NUM_CLASS);//同上，获取类别索引
        float class_conf = output_buffer[basic_pos + 9 + NUM_COLORS + class_idx];

        //获取置信度
        float box_conf = output_buffer[basic_pos + 8];

        if (box_conf>prob_threshold) {
            Object obj;
            obj.p1 = cv::Point2f(x1, y1);
            obj.p2 = cv::Point2f(x2, y2);
            obj.p3 = cv::Point2f(x3, y3);
            obj.p4 = cv::Point2f(x4, y4);
            //用关键点算出检测框，虽然我们模型里没有检测框，但为了后面非极大抑制还是得算出来
            std::vector<cv::Point2f> rect_pts;
            rect_pts.emplace_back(obj.p1);
            rect_pts.emplace_back(obj.p2);
            rect_pts.emplace_back(obj.p3);
            rect_pts.emplace_back(obj.p4);

            obj.rect = cv::boundingRect(rect_pts);

            obj.label = class_idx;
            obj.color = color_idx * 9;
            //分类置信度和识别置信度的乘积才是最后真正算出来的置信度
            obj.confidence = box_conf * ((class_conf + color_conf) / 2);

            objects.push_back(obj);
        }
    }
}

void NetArmorDetector::qsort_descent_inplace(std::vector<Object> & faceobjects, int left, int right) {
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].confidence;
    while (i <= j) {
        while (faceobjects[i].confidence > p) {
            i++;
        }

        while (faceobjects[j].confidence < p) {
            j--;
        }

        if (i <= j) {
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if(left < j) {
                qsort_descent_inplace(faceobjects, left, j);
            }
        }
        #pragma omp section
        {
            if(i < right) {
                qsort_descent_inplace(faceobjects, i, right);
            }
        }
    }
}

void NetArmorDetector::qsort_descent_inplace(std::vector<Object>& objects) {
    if (objects.empty()) {
        return;
    }
    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void NetArmorDetector::nms_sorted_bboxes(std::vector<Object> & faceobjects, std::vector<int>& picked, float nms_threshold) {
    picked.clear();
    const int n = faceobjects.size();
    std::vector<float> areas(n);

    for (int i = 0; i < n; i++) {
        //计算每一个面积
        std::vector<cv::Point2f> object_apex_tmp;
        object_apex_tmp.emplace_back(faceobjects[i].p1);
        object_apex_tmp.emplace_back(faceobjects[i].p2);
        object_apex_tmp.emplace_back(faceobjects[i].p3);
        object_apex_tmp.emplace_back(faceobjects[i].p4);
        areas[i] = cv::contourArea(object_apex_tmp);
    }

    for (int i = 0; i < n; i++) {
        Object& a = faceobjects[i];
        std::vector<cv::Point2f>apex_a;
        apex_a.emplace_back(a.p1);
        apex_a.emplace_back(a.p2);
        apex_a.emplace_back(a.p3);
        apex_a.emplace_back(a.p4);

        int keep = 1;

        for (int j = 0; j < static_cast<int>(picked.size()); j++) {
            Object &b = faceobjects[picked[j]];
            //不是一类的就不要方一起比较了
            if (a.color != b.color || a.label != b.label) {
                continue;
            }
            std::vector<cv::Point2f>apex_b, apex_inter;
            apex_b.emplace_back(b.p1);
            apex_b.emplace_back(b.p2);
            apex_b.emplace_back(b.p3);
            apex_b.emplace_back(b.p4);

            float inter_area = cv::intersectConvexConvex(apex_a, apex_b, apex_inter);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            float iou = inter_area / union_area;

            if (iou > nms_threshold || isnan(iou)) {
                keep=0;
                if(iou>0.2 && abs(a.confidence - b.confidence) < 0.15 && 
                    a.label == b.label && a.color == b.color) {
                    b.p1 = a.p1;
                    b.p2 = a.p2;
                    b.p3 = a.p3;
                    b.p4 = a.p4;
                }
            }
        }
        if (keep) {
            picked.emplace_back(i);
        }
    }

}

void NetArmorDetector::decode(const float* output_buffer, std::vector<Object>& objects, float scale) {
    std::vector<Object>proposals;
    const int strides[3] = {8, 16, 32};//步长
    std::vector<GridAndStride> grid_strides;
    generate_grids_and_stride(cam_info_->width, cam_info_->height, strides, grid_strides);
    generate_yolox_proposal(grid_strides, output_buffer, 0.7, proposals, scale);
    //std::cout<<proposals.size()<<std::endl;
    qsort_descent_inplace(proposals);
    if (proposals.size() >= 128) {
        proposals.resize(128);
    }

    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, NMS_THRESH);

    int count = picked.size();
    objects.resize(count);
    //非极大抑制后的放入object里
    for (int i = 0; i < count; i++) {
        objects[i] = proposals[picked[i]];
    }
}

} // helios_cv