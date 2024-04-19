#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

//试图变换——摆相机
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//模型变换——摆位置(由于此实验直接在场景中画出来，所以只需要处理旋转部分)
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    rotation_angle=rotation_angle/180.0f*MY_PI;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // model<<1,0,0,0,
    //         0,cos(rotation_angle),-sin(rotation_angle),0,
    //         0,sin(rotation_angle),cos(rotation_angle),0,
    //         0,0,0,1;
    // model<<cos(rotation_angle),0,sin(rotation_angle),0,
    //         0,1,0,0,
    //         -sin(rotation_angle),0,cos(rotation_angle),0,
    //         0,0,0,1;
    model<<cos(rotation_angle),-sin(rotation_angle),0,0,
           sin(rotation_angle),cos(rotation_angle),0,0,
           0,0,1,0,
           0,0,0,1;
    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    angle=angle/180.0f*MY_PI;
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result<<cos(angle)+axis[0]*axis[0]*(1-cos(angle)),axis[0]*axis[1]*(1-cos(angle))-axis[2]*sin(angle),axis[0]*axis[2]*(1-cos(angle))+axis[1]*sin(angle),0,
            axis[1]*axis[0]*(1-cos(angle))+axis[2]*sin(angle),cos(angle)+axis[1]*axis[1]*(1-cos(angle)),axis[1]*axis[2]*(1-cos(angle))-axis[0]*sin(angle),0,
            axis[2]*axis[0]*(1-cos(angle))-axis[1]*sin(angle),axis[2]*axis[1]*(1-cos(angle))+axis[0]*sin(angle),cos(angle)+axis[2]*axis[2]*(1-cos(angle)),0,
            0,0,0,1;
    return result;
}

//投影变换——拍照
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    eye_fov=eye_fov/180*MY_PI;
    projection<<1/(aspect_ratio*tan(eye_fov/2.0f)) ,0,0,0,
                0,1/tan(eye_fov/2.0f),0,0,
                0,0,(zFar+zNear)/(zFar-zNear),-2*zFar*zNear/(zNear-zFar),
                0,0,1,0;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    Vector3f axis = Vector3f(0,0,1);
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(800, 800);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        std::cout << "command_line true" << std::endl; 
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    std::cout << "command_line false" << std::endl; 
    while (key != 27) {  //27Esc
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(800, 800, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("Triangle", image);
        key = cv::waitKey(100);

        if (key == 'a') {
            angle += 5;
        }
        else if (key == 'd') {
            angle -= 5;
        }
    }

    return 0;
}