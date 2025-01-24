// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

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
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model = get_rotation(Vector3f(1, 1, 0), rotation_angle);
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection;
 Matrix4f persp2ortho, ortho;

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
    // 3. 计算长方体 压缩成 -1 1 的标准正方体
    ortho <<
            2 / width, 0, 0, 0,
            0, 2 / height, 0, 0,
            0, 0, 2 / (n - f), 0,
            0, 0, 0, 1;

    projection = ortho * persp2ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on
