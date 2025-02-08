//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H

#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name) {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v) {
        // 1. 传入的 u，v存在 不在 0，1范围内，处理一下。
        u = std::min(1.0f, std::max(0.0f, u));
        v = std::min(1.0f, std::max(0.0f, v));

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBiilinear(float u, float v) {
        u = std::min(1.0f, std::max(0.0f, u));
        v = std::min(1.0f, std::max(0.0f, v));

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        auto u_min = std::floor(u_img);
        auto u_max = std::ceil(u_img);
        auto v_min = std::floor(v_img);
        auto v_max = std::ceil(v_img);

        auto color_11 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto color_12 = image_data.at<cv::Vec3b>(v_min, u_max);
        auto color_21 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto color_22 = image_data.at<cv::Vec3b>(v_max, u_max);

        float s = (u_img - u_min) / (u_max - u_min);
        auto color1 = color_11 + s * (color_12 - color_11);
        auto color2 = color_21 + s * (color_22 - color_21);

        float t = (v_img - v_min) / (v_max - v_min);
        auto color = color1 + t * (color2 - color1);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};

#endif //RASTERIZER_TEXTURE_H
