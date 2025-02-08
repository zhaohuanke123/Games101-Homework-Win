#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

constexpr float DegreeToRadian = MY_PI / 180;

/**
 * @param axis 过原点的旋转轴
 * @param angle 旋转角度
 * @return 过原点的任意轴旋转任意角度的齐次旋转矩阵
 */
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    // 规一化旋转轴
    axis = axis.normalized();
    float alpha = angle * DegreeToRadian;

    Matrix3f multFactor;
    // 2. 计算叉乘矩阵
    multFactor <<
            0, -axis.z(), axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0;

    // 3. 代入公式计算旋转矩阵
    Matrix3f rotation = cos(alpha) * Matrix3f::Identity()
                        + (1 - cos(alpha)) * axis * axis.transpose()
                        + sin(alpha) * multFactor;
    Matrix4f final = Matrix4f::Identity();

    // 4. 转换为 齐次矩阵
    final.block(0, 0, 3, 3) = rotation;
    return final;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate <<
            1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;
    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // const float theta = rotation_angle * DegreeToRadian;
    // Matrix4f rotate;
    // rotate <<
    //         cos(theta), -sin(theta), 0, 0,
    //         sin(theta), cos(theta), 0, 0,
    //         0, 0, 1, 0,
    //         0, 0, 0, 1;
    // model = rotate * model;
    // 1. 绕任意轴的旋转
    model = get_rotation(Vector3f(1, 1, 0), rotation_angle);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    projection = Eigen::Matrix4f::Identity();
    Matrix4f persp2ortho, scale, translate;

    // 1. 计算从视锥体压缩到长方体的矩阵
    float n = -zNear;
    float f = -zFar;
    persp2ortho <<
            n, 0, 0, 0,
            0, n, 0, 0,
            0, 0, n + f, -n * f,
            0, 0, 1, 0;

    // 2. 计算长方体的各个参数
    float theta = eye_fov * 0.5 * DegreeToRadian;
    float height = zNear * tan(theta) * 2;
    float width = height * aspect_ratio;

    // 3. 计算长方体 压缩成 -1 1 的标准正方体  平移 + 缩放
    scale <<
            2 / width, 0, 0, 0,
            0, 2 / height, 0, 0,
            0, 0, 2 / (n - f), 0,
            0, 0, 0, 1;
    translate <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -(f + n) / (n - f),
            0, 0, 0, 1;

    Matrix4f ortho = scale * translate;
    projection = ortho * persp2ortho;

    return projection;
}

int main(int argc, const char **argv) {
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        } else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    // 点集合
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 索引集合
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    // 输入的按键key
    int key = 0;
    // 当前渲染了多少帧
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 设置MVP矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);
        std::cout << filename << std::endl;

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        } else if (key == 'd') {
            angle -= 10;
        } else if (key == 'w') {
            eye_pos.z() -= 1;
        } else if (key == 's') {
            eye_pos.z() += 1;
        }
    }

    return 0;
}
