#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tchar.h>
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib,"ws2_32.lib") // Winsock Library
#include <windows.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "utils.h"

// using namespace std;
// using namespace cv;

#define BUFLEN 512
#define PORT 10086

#define ROW 25
#define COL 20

typedef struct {
    int a, b;
} pair_t;

const float show_scale = 0.25;
int show_pos = 0;

// my imshow
void my_image_show(const char *name, const cv::Mat &mat, float show_scale_ = show_scale) {
    // cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::Mat buf;
    if (strcmp(name, "perspective") == 0) {
        cv::resize(mat, buf, cv::Size((int)mat.cols * show_scale * 2, (int)mat.rows * show_scale * 2), 0, 0, cv::INTER_NEAREST);
    } else {
        // printf("%d, %d", mat.cols, mat.rows);
        cv::resize(mat, buf, cv::Size((int)mat.cols * show_scale_, (int)mat.rows * show_scale_), 0, 0, cv::INTER_NEAREST);
    }
    cv::imshow(name, buf);
    cv::setWindowTitle(name, show_pos == -1 ? cv::format("%s: live\n", name) : cv::format("%s: %d\n", name, show_pos));
}

static double my_perspective_m[3][3] = {
    // 1.079859758472925, 1.410206466692636, -26.54148811842614,
    // -6.134181891454103e-16, 4.078691079080636, 6.229061160888224,
    // -6.279874685452773e-18, 0.0307752239968835, 1
    // 8.852852852852793, 20.91891891891879, -555.309309309305,
    // -1.240220047872175e-14, 45.94594594594567, -106.3063063063054,
    // -3.314982104985805e-17, 0.2072072072072059, 1
    1.614820944642267, 6.410692148080227, -48.47086214702354,
    -0.9504804035396475, 15.78726638586437, 5.840723707954306,
    -0.004752402017698242, 0.06444683369857217, 1
};

// 逆透视变换
static inline pair_t my_perspective_transform(pair_t in) {
    // Ref: https://docs.opencv.org/4.x/d2/de8/group__core__array.html#gad327659ac03e5fd6894b90025e6900a7

    // double a[1][3] = { in.b, in.a, 1 }; // x, y, 1
    // double b[1][3] = {};
    // for (int i = 0; i < 1; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         for (int k = 0; k < 3; ++k) {
    //             b[i][j] += a[i][k] * my_perspective_m[j][k]; // 与标准矩阵相比，my_perspective_m矩阵需要转置一下再参与运算
    //         }
    //     }
    // }
    // b[0][0] /= b[0][2];
    // b[0][1] /= b[0][2];
    // b[0][2] /= b[0][2];
    // std::cout << b[0][0] << " " << b[0][1] << " " << b[0][2] << " " << std::endl;
    // pair_t out = { b[0][1], b[0][0] };

    double x1 = in.b, y1 = in.a, w1 = 1;
    double x2 = x1 * my_perspective_m[0][0] + y1 * my_perspective_m[0][1] + w1 * my_perspective_m[0][2],
        y2 = x1 * my_perspective_m[1][0] + y1 * my_perspective_m[1][1] + w1 * my_perspective_m[1][2],
        w2 = x1 * my_perspective_m[2][0] + y1 * my_perspective_m[2][1] + w1 * my_perspective_m[2][2];
    pair_t out = { (int)(y2 / w2), (int)(x2 / w2) };
    return out;
}

// 计算经过逆透视变换后的真实距离
// 这个函数很耗时间，尽量不要使用
static inline int my_real_distance(pair_t in1, pair_t in2) {
    pair_t out1 = my_perspective_transform(in1);
    pair_t out2 = my_perspective_transform(in2);
    // printf("%d %d %d %d\n", out1.a, out1.b, out2.a, out2.b);
    return sqrt((out1.a - out2.a) * (out1.a - out2.a) + (out1.b - out2.b) * (out1.b - out2.b));
}

typedef struct {
    const char *name;
    const char *pers_name;
    cv::Mat image;
    cv::Mat pers_image;
    cv::RNG rng;
    pair_t last_p;
    pair_t last_pers_p;
    int pic_num;
} my_mouse_data_t;
static my_mouse_data_t my_mouse_data;

static int point_num = 0;
pair_t origin_point[4] = {0};

void homography_compute2(float_t dst[3][3], const float_t c[4][4]) {
    // clang-format off
    float_t A[]{
        c[0][0], c[0][1], 1,       0,       0, 0, -c[0][0]*c[0][2], -c[0][1]*c[0][2], c[0][2],
              0,       0, 0, c[0][0], c[0][1], 1, -c[0][0]*c[0][3], -c[0][1]*c[0][3], c[0][3],
        c[1][0], c[1][1], 1,       0,       0, 0, -c[1][0]*c[1][2], -c[1][1]*c[1][2], c[1][2],
              0,       0, 0, c[1][0], c[1][1], 1, -c[1][0]*c[1][3], -c[1][1]*c[1][3], c[1][3],
        c[2][0], c[2][1], 1,       0,       0, 0, -c[2][0]*c[2][2], -c[2][1]*c[2][2], c[2][2],
              0,       0, 0, c[2][0], c[2][1], 1, -c[2][0]*c[2][3], -c[2][1]*c[2][3], c[2][3],
        c[3][0], c[3][1], 1,       0,       0, 0, -c[3][0]*c[3][2], -c[3][1]*c[3][2], c[3][2],
              0,       0, 0, c[3][0], c[3][1], 1, -c[3][0]*c[3][3], -c[3][1]*c[3][3], c[3][3],
    };
    // clang-format on

    // Eliminate.
    for (int col = 0; col < 8; col++) {
        // Find best row to swap with.
        float_t max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++) {
            float_t val = fabs(A[row * 9 + col]);
            if (val > max_val) {
                max_val = val;
                max_val_idx = row;
            }
        }

        // Swap to get best row.
        if (max_val_idx != col) {
            for (int i = col; i < 9; i++) {
                float_t tmp = A[col * 9 + i];
                A[col * 9 + i] = A[max_val_idx * 9 + i];
                A[max_val_idx * 9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++) {
            float_t f = A[i * 9 + col] / A[col * 9 + col];
            A[i * 9 + col] = 0;
            for (int j = col + 1; j < 9; j++) A[i * 9 + j] -= f * A[col * 9 + j];
        }
    }

    // Back solve.
    for (int col = 7; col >= 0; col--) {
        float_t sum = 0;
        for (int i = col + 1; i < 8; i++) { sum += A[col * 9 + i] * A[i * 9 + 8]; }
        A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
    }
    // clang-format off
    dst[0][0] = A[8 ], dst[0][1] = A[17], dst[0][2] = A[26];
    dst[1][0] = A[35], dst[1][1] = A[44], dst[1][2] = A[53];
    dst[2][0] = A[62], dst[2][1] = A[71], dst[2][2] = 1;
    // clang-format on
}

// 画布上鼠标操作处理函数
// event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
static void on_mouse(int event, int x, int y, int flags, void *raw_data) {
    my_mouse_data_t *data = (my_mouse_data_t *)raw_data;
    switch (event) {
    case cv::EVENT_LBUTTONDOWN: // 按下左键
        pair_t p;
        if (data->pic_num == 1) {
            p.a = (int)(y / show_scale / 1.5);
            p.b = (int)(x / show_scale / 1.5);
        } else if (data->pic_num == 2) {
            p.a = (int)(y / show_scale / 1.2);
            p.b = (int)(x / show_scale / 1.2);
        } else {
            p.a = (int)(y / show_scale);
            p.b = (int)(x / show_scale);
        }

        int len = sqrt(pow(data->last_p.a - p.a, 2) + pow(data->last_p.b - p.b, 2));

        pair_t pers_p = my_perspective_transform(p);
        int pers_len = my_real_distance(data->last_pers_p, pers_p);

        std::cout << cv::format("(%d,%d) %d (%d,%d) %d", p.a, p.b, len, pers_p.a, pers_p.b, pers_len) << std::endl;

        cv::Scalar color(data->rng.uniform(0, 255), data->rng.uniform(0, 255), data->rng.uniform(0, 255));

        cv::Point p_cv(p.b, p.a);
        cv::putText(data->image, cv::format("(%d,%d)", p.a, p.b), p_cv, cv::FONT_HERSHEY_SIMPLEX, 0.25, color);
        cv::circle(data->image, p_cv, 1, color, 2, 8, 0);
        if (data->pic_num == 1) {
            my_image_show(data->name, data->image, show_scale * 1.5); // 刷新图片
        } else if (data->pic_num == 2) {
            my_image_show(data->name, data->image, show_scale * 1.2); // 刷新图片
        } else {
            my_image_show(data->name, data->image, show_scale); // 刷新图片
        }

        // cv::Point pers_p_cv(pers_p.b, pers_p.a);
        // cv::putText(data->pers_image, cv::format("(%d,%d)", pers_p.a, pers_p.b), pers_p_cv, cv::FONT_HERSHEY_SIMPLEX, 0.25, color);
        // cv::circle(data->pers_image, pers_p_cv, 1, color, 2, 8, 0);

        origin_point[point_num++] = {p.a, p.b};
        point_num = (point_num < 4) ? point_num : 0;

        /*
            获取透视变换用矩阵（标定）
            车辆放在长直道，在画面内可见的赛道两侧边缘，远端与近端放置共计4个标志
            在图像中测量4个标志的像素点坐标，并且在现实中也测量4个标志的物理位置坐标（可以以mm为最小单位）
            将8个位置信息填入下表即可得出变换矩阵
        */
        cv::Point2f src_points[] = {
            cv::Point2f(origin_point[0].b, origin_point[0].a), // （列，行）
            cv::Point2f(origin_point[1].b, origin_point[1].a),
            cv::Point2f(origin_point[2].b, origin_point[2].a),
            cv::Point2f(origin_point[3].b, origin_point[3].a)
        };

        // (888,940) 1293 (-87,159) 48
        // (1684,406) 958 (-1124,565) 1952 // 左侧上到下
        // (1614,3096) 2690 (-1,142) 1918
        // (794,2444) 1047 (15,131) 6 // 右侧下到上

        cv::Point2f dst_points[4] = {};
        if (data->pic_num == 1) {
            dst_points[0] = cv::Point2f(816 - 170, 600);
            dst_points[1] = cv::Point2f(816 - 170, 1100);
            dst_points[2] = cv::Point2f(816 + 170, 1100);
            dst_points[3] = cv::Point2f(816 + 170, 600);
        } else if (data->pic_num == 2) {
            dst_points[0] = cv::Point2f(816 - 180, 600);
            dst_points[1] = cv::Point2f(816 - 180, 1100);
            dst_points[2] = cv::Point2f(816 + 180, 1100);
            dst_points[3] = cv::Point2f(816 + 180, 600);
        } else {
            dst_points[0] = cv::Point2f(816 - 200, 600);
            dst_points[1] = cv::Point2f(816 - 200, 1100);
            dst_points[2] = cv::Point2f(816 + 200, 1100);
            dst_points[3] = cv::Point2f(816 + 200, 600);
        }

        cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);

        // float dst[3][3] = {0};
        // float points[4][4]= {};
        // if (data->pic_num == 1) {
        //     points[0][0] = 816 - 170;
        //     points[0][1] = 1100;
        //     points[1][0] = 816 + 170;
        //     points[1][1] = 600;
        //     points[2][0] = 816 + 170;
        //     points[2][1] = 600;
        //     points[3][0] = 816 + 170;
        //     points[3][1] = 600;
        // } else if (data->pic_num == 2) {
        //     // points[0] = cv::Point2f(816 - 180, 600);
        //     // points[1] = cv::Point2f(816 - 180, 1100);
        //     // points[2] = cv::Point2f(816 + 180, 1100);
        //     // points[3] = cv::Point2f(816 + 180, 600);
        //     points[0][0] = 816 - 180;
        //     points[0][1] = 1100;
        //     points[1][0] = 816 + 180;
        //     points[1][1] = 600;
        //     points[2][0] = 816 + 180;
        //     points[2][1] = 600;
        //     points[3][0] = 816 + 180;
        //     points[3][1] = 600;
        // } else {
        //     // points[0] = cv::Point2f(816 - 200, 600);
        //     // points[1] = cv::Point2f(816 - 200, 1100);
        //     // points[2] = cv::Point2f(816 + 200, 1100);
        //     // points[3] = cv::Point2f(816 + 200, 600);
        //     points[0][0] = 816 - 200;
        //     points[0][1] = 1100;
        //     points[1][0] = 816 + 200;
        //     points[1][1] = 600;
        //     points[2][0] = 816 + 200;
        //     points[2][1] = 600;
        //     points[3][0] = 816 + 200;
        //     points[3][1] = 600;
        // }

        // points[0][2] = origin_point[0].b;
        // points[0][3] = origin_point[0].a;
        // points[1][2] = origin_point[1].b;
        // points[1][3] = origin_point[1].a;
        // points[2][2] = origin_point[2].b;
        // points[2][3] = origin_point[2].a;
        // points[3][2] = origin_point[3].b;
        // points[3][3] = origin_point[3].a;

        // homography_compute2(dst, points);
        // cv::Mat M = cv::Mat(3, 3, CV_32F, dst);

        // std::cout << M << std::endl;
        // std::cout << M.at<double>(0, 1) << std::endl; // 用于辅助识别输出的行列
        memcpy(my_perspective_m, M.isContinuous() ? M.data : M.clone().data, sizeof my_perspective_m);

        cv::warpPerspective(data->image, data->pers_image, cv::Mat(3, 3, CV_64F, my_perspective_m), cv::Size(408 * 4, 408 * 3), cv::INTER_LINEAR);
        my_image_show(data->pers_name, data->pers_image); // 刷新图片

        data->last_p = p;
        data->last_pers_p = pers_p;
        break;
    }
}

void draw_line(cv::Mat frame, int start_x, int start_y, int end_x, int end_y, cv::Scalar CV_COLOR) {
    cv::Point start = cv::Point(start_x, start_y); //直线起点
    cv::Point end = cv::Point(end_x, end_y); //直线终点
    cv::line(frame, start, end, CV_COLOR);
}

typedef struct {
    int x;
    int y;
} position_t;

void my_popen(const char *cmd, char cmd_result[], int cmd_result_size, int *cmd_result_len) {
    FILE *fp = _popen(cmd, "r");
    if (fp == NULL) {
        // error
    }

    cmd_result[0] = '\0'; // initialize the big buffer to an empty string
    *cmd_result_len = 0;

#define BUF_SIZE 1234
    static char buf[BUF_SIZE];
    while (fgets(buf, BUF_SIZE, fp)) {
        size_t len = strlen(buf);
        if (*cmd_result_len + len >= cmd_result_size) {
            // error
        }

        strcat(cmd_result, buf);
        *cmd_result_len += strlen(buf);
#undef BUF_SIZE
    }

    _pclose(fp);
}

#define CMD_RESULT_SIZE 600
char cmd_result[CMD_RESULT_SIZE];
int cmd_result_len;

void adb_exec(const char *device, const char *command) {
    static char cmd[500] = {};
    sprintf(cmd, "adb -s %s %s", device, command);
    my_popen(cmd, cmd_result, CMD_RESULT_SIZE, &cmd_result_len);
    printf("%s\n", cmd_result);
}

int main(int argc, char* argv[]) {
    setbuf(stdout, NULL);
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

    int num[2] = { 0, 0 }; // 图片总数
    position_t position[3][30] = {}; // 图片位置

    // 创建套接字 绑定
    sockaddr_in server, client;

    // initialise winsock
    WSADATA wsa;
    printf("Initialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("Failed. Error Code: %d", WSAGetLastError());
        exit(0);
    }
    printf("Initialised.\n");

    // create a socket
    SOCKET server_socket;
    if ((server_socket = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
        printf("Could not create socket: %d", WSAGetLastError());
    }
    printf("Socket created.\n");

    // prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    // server.sin_addr.s_addr = inet_addr("192.168.43.161");
    // server.sin_addr.s_addr = inet_addr("0.0.0.0");
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(PORT);

    // bind
    if (bind(server_socket, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        printf("Bind failed with error code: %d", WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Bind done.\n");

    while (1) {
        printf("mode choose:\n");
        char mode[10];
        scanf_s("%s", mode, 10);
        if (strcmp(mode, "recv") == 0) {
            char *devices[] = { // adb devices
                "DLS4C20716005735", // 华为
                "192.168.43.96", // 小米
            };

            // 打开摄像头
            adb_exec(devices[0], "shell am start -n com.huawei.camera/com.huawei.camera"); // 华为
            adb_exec(devices[1], "shell am start -n com.android.camera/.Camera"); // 小米
            Sleep(500);

            // 拍照
            for (int i = 0; i < 2; ++i) {
                adb_exec(devices[i], "shell input keyevent 24"); // 第一次用于唤醒相机
                adb_exec(devices[i], "shell input keyevent 24");
            }
            Sleep(4000); // 由于手机图像处理以及存盘过程较慢？ 所以delay 4秒

            for (int i = 0; i < 2; ++i) {
                // 获取路径
                adb_exec(devices[i], "shell ls -t /sdcard/DCIM/Camera/IMG_20220822*"); // 列出目录信息
                char filepath[100];
                char *end = strchr(cmd_result, '\n');
                if (end != NULL) {
                    memcpy(filepath, cmd_result, (end - cmd_result) * sizeof cmd_result[0]);
                    filepath[end - cmd_result] = 0;
                } else {
                    // error
                }
                printf("image path to get: %s\n", filepath);

                // 接收图像
                char cmd[500];
                sprintf(cmd, "pull %s ./recv/%d.jpg", filepath, i + 1);
                adb_exec(devices[i], cmd);
            }
        } else if (strcmp(mode, "calc") == 0) {
            while (1) {
                printf("which picture?\n");
                int pic_num;
                scanf("%d", &pic_num);
                char pic_dir[100];
                sprintf(pic_dir, "./recv/%d.jpg", pic_num);

                // 读取图片
                cv::Mat frame_read = cv::imread(pic_dir, cv::IMREAD_COLOR);
                cv::Mat debug_image_perspective;
                cv::warpPerspective(frame_read, debug_image_perspective, cv::Mat(3, 3, CV_64F, my_perspective_m), cv::Size(408 * 4, 408 * 3), cv::INTER_LINEAR);

                // 显示原图 以及用默认矩阵算得的逆透视图像
                if (pic_num == 1) {
                    my_image_show("1", frame_read, show_scale * 1.5);
                } else if (pic_num == 2) {
                    my_image_show("1", frame_read, show_scale * 1.2);
                } else {
                    my_image_show("1", frame_read);
                }
                my_image_show("perspective", debug_image_perspective);

                // 设置鼠标时间回调函数
                my_mouse_data.rng = cv::RNG(12345); // 固定随机数种子使得每次颜色差不多
                my_mouse_data.name = "1";
                my_mouse_data.pic_num = pic_num;
                my_mouse_data.image = frame_read; // 潜拷贝 .clone() 为深拷贝
                my_mouse_data.pers_name = "perspective";
                my_mouse_data.pers_image = debug_image_perspective; // 潜拷贝 .clone() 为深拷贝
                cv::setMouseCallback(my_mouse_data.name, on_mouse, &my_mouse_data); // 通过 userdata 传递画布

                // 保存矩阵 或 重新标定
                int key = cv::waitKey(0);
                if (key == 'y') {
                    char matrix_dir[100];
                    sprintf(matrix_dir, "./matrix%d.txt", pic_num);

                    // 打开文件，准备写入
                    FILE *fp;
                    if (fopen_s(&fp, matrix_dir, "wb") != 0) {
                        printf("file open failed: matrix%d.txt\n", pic_num);
                        exit(1);
                    }
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            if (fprintf(fp, "%f ", my_perspective_m[i][j]) < 0) {
                                printf("file write failed: matrix%d.txt\n", pic_num);
                                break;
                            }
                            // printf("%f", my_perspective_m[i][j]);
                        }
                    }
                    printf("matrix%d saved in matrix%d.txt\n", pic_num, pic_num);
                    fclose(fp);

                    cv::destroyAllWindows();
                    break;
                } else if (key == 'n') {
                    printf("remake");
                    cv::destroyAllWindows();
                    continue;
                } else {
                    cv::destroyAllWindows();
                    break;
                }
            }
        } else if (strcmp(mode, "handle") == 0) {
            int pic_num = 0;
            // printf("which picture?\n");
            for (int picture_number = 1; picture_number <= 2; ++picture_number) {
                // scanf("%d", &pic_num);
                pic_num = picture_number;
                char pic_dir[100];
                sprintf(pic_dir, "./recv/%d.jpg", pic_num);
                char matrix_dir[100];
                sprintf(matrix_dir, "./matrix%d.txt", pic_num);

                // 打开文件，准备读取
                FILE *fp;
                if (fopen_s(&fp, matrix_dir, "r") != 0) {
                    printf("file open failed: matrix%d.txt\n", pic_num);
                    exit(1);
                }
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        fscanf(fp, "%lf ", &my_perspective_m[i][j]);
                    }
                }
                fclose(fp);
                // for (int i = 0; i < 3; ++i) {
                //     for (int j = 0; j < 3; ++j) {
                //         printf("%f ", my_perspective_m[i][j]);
                //     }
                // }

                // 根据保存的矩阵做逆透视 输出图像大小为(408 * 4, 408 * 3)
                cv::Mat origin_frame = cv::imread(pic_dir, cv::IMREAD_COLOR);
                cv::Mat perspective_frame;
                cv::warpPerspective(origin_frame, perspective_frame, cv::Mat(3, 3, CV_64F, my_perspective_m), cv::Size(408 * 4, 408 * 3), cv::INTER_LINEAR);

                // 通道分离
                cv::Mat frame_splited[4]; // 通道分离用frame
                cv::split(perspective_frame, frame_splited);
                const char *name_group[4] = {"blue", "green", "red", "green+red"};
                int channel;
                // printf("which channel?\n");
                // printf("1:blue 2:green 3:red 4:green+red\n");
                // scanf("%d", &channel);
                channel = 3; // 先默认为红
                channel -= 1;
                if (channel == 3) {
                    frame_splited[channel] = (frame_splited[1] + frame_splited[2]) / 2;
                }

                // 图像转换为数组
                static unsigned char img[408 * 3][408 * 4];
                memcpy(img, frame_splited[channel].isContinuous() ? frame_splited[channel].data : frame_splited[channel].clone().data, sizeof img); // Mat 转数组

                // handle
                // 直接用2维
                typedef struct {
                    unsigned char pic[20][20];
                    unsigned char avg;
                } block_t;
                block_t block[ROW][COL]; // COL 已经不用 只作为定义时开辟空间

                pic_num -= 1; // 自此以后 pic_num 作为数组下标使用
                int col[2] = {17, 18};

                // 分割和判别计算
                int start_y[2] = {646, 636};
                int total = 0; // 一块中的总灰度
                num[pic_num] = 0;
                memset(position[pic_num], 0, sizeof position[pic_num]); // 清零
                for (int i = 0; i < ROW; ++i) {
                    // printf("\n");
                    for (int j = 0; j < col[pic_num]; ++j) {
                        // 每一块的起始点二维数组坐标 算出来
                        // [0][0] 对应img[600][616]
                        total = 0;
                        for (int k = 0; k < 20; ++k) {
                            for (int l = 0; l < 20; ++l) {
                                block[i][j].pic[k][l] = img[600 + i * 20 + k][start_y[pic_num] + j * 20 + l];
                                total += block[i][j].pic[k][l];
                            }
                        }
                        block[i][j].avg = total / 400;

                        // 灰度判断法
                        // if (block[i][j].avg > 80) {
                        //     printf(ANSI_COLOR_RED "%3d " ANSI_COLOR_RESET, block[i][j].avg);
                        //     position[pic_num][num].x = i;
                        //     position[pic_num][num].y = j;
                        //     num++;
                        // } else {
                        //     printf("%3d ", block[i][j].avg);
                        // }

                        // 显示一下 看分割的对不对 // 没问题
                        // cv::Mat show;
                        // show.create(20, 20, CV_8UC1);
                        // memcpy(show.data, (&block[i][j])->pic, 400);
                        // cv::imshow(cv::format("%d, %d", i, j), show);
                    }
                }

                // 差值判断法
                for (int i = 0; i < ROW; ++i) {
                    printf("\n");
                    for (int j = 0; j < col[pic_num]; ++j) {
                        int round_avg = 0;
                        int diff = 0;

                        // 1 2 3
                        // 0   4
                        // 7 6 5
                        position_t dir[8] = {{0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}};

                        int n = 0;
                        for (int k = 0; k < 8; ++k) {
                            int x = i + dir[k].x;
                            int y = j + dir[k].y;
                            if (x < 0 || y < 0 || x > ROW - 1 || y > col[pic_num] - 1) continue;
                            round_avg += block[x][y].avg;
                            ++n;
                        }
                        round_avg /= n;
                        diff = block[i][j].avg - round_avg;
                        if (diff > 30) {
                            printf(ANSI_COLOR_RED "%2d " ANSI_COLOR_RESET, diff);
                            position[pic_num][num[pic_num]].x = j;
                            position[pic_num][num[pic_num]].y = i;
                            num[pic_num]++;

                        } else {
                            printf("%2d ", diff);
                        }
                    }
                }

                for (int i = 0; i < num[pic_num]; ++i) { // 坐标转化第一步 将自己脚底下变成(0, 0)
                    position[pic_num][i].x += 1;
                    position[pic_num][i].y = 24 - position[pic_num][i].y + 1;
                }

                printf("\ntotal: %d\n", num[pic_num]);
                for (int i = 0; i < num[pic_num]; ++i) {
                    printf("[%d, %d] ", position[pic_num][i].x, position[pic_num][i].y);
                }
                printf("\n");

                // 显示
                int x[3] = {646, 636};
                int y = 1100;
                for (int i = 0; i < 21; ++i) {
                    draw_line(frame_splited[channel], x[pic_num], 0, x[pic_num], 408 * 3, cv::Scalar(255, 255, 255));
                    x[pic_num] += 20;
                }
                for (int i = 0; i < 26; ++i) {
                    draw_line(frame_splited[channel], 0, y, 408 * 4, y, cv::Scalar(255, 255, 255));
                    y -= 20;
                }
                cv::Mat frame_show[4]; // 显示用frame
                cv::resize(frame_splited[channel], frame_show[channel], cv::Size((int)frame_splited[channel].cols * show_scale * 2, (int)frame_splited[channel].rows * show_scale * 2), 0, 0, cv::INTER_NEAREST);
                cv::imshow(name_group[channel], frame_show[channel]);
                cv::setWindowTitle(name_group[channel], name_group[channel]);
                cv::waitKey(0);
            }

            // 坐标转换第二步 转换图2坐标并拼接
            for (int i = 0; i < num[1]; ++i) { // 坐标转换
                position[1][i].x = 18 - position[1][i].x + 1 + 17;
                position[1][i].y = 25 - position[1][i].y + 1;
                // printf("(%d, %d)\n", position[1][i].x, position[1][i].y);
            }

            // 拼接
            for (int i = 0; i < num[0]; ++i) {
                position[2][i] = position[0][i];
            }
            for (int i = 0; i < num[1]; ++i) {
                position[2][num[0] + i] = position[1][i];
            }

            for (int i = 0; i < num[0] + num[1]; ++i) {
                printf("(%d, %d)\n", position[2][i].x, position[2][i].y);
            }
            cv::waitKey(0);
            cv::destroyAllWindows();
            continue;
        } else if (strcmp(mode, "send") == 0) { // from https://gist.github.com/sunmeat/02b60c8a3eaef3b8a0fb3c249d8686fd
            for (int car_num = 0; car_num < 2; car_num++) {
                // 将位置结构体存成数组
                char message[BUFLEN] = {};
                // message[0] = num[car_num];
                // for (int i = 0; i < num[car_num]; ++i) { // x 轴
                //     int n = 2 * (i + 1) - 1;
                //     // message[n] = 25 - position[car_num][i].x + 1;
                //     message[n] = position[car_num][i].y + 1;
                // }
                // for (int i = 0; i < num[car_num]; ++i) { // y 轴
                //     int n = 2 * (i + 1);
                //     // message[n] = position[car_num][i].y + 1;
                //     message[n] = 24 - position[car_num][i].x + 1;
                // }

                for (int i = 0; i < num[0] + num[1]; ++i) {
                    printf("(%d, %d)\n", position[2][i].x, position[2][i].y);
                }
                int total_num = 0;
                if (car_num == 0) {
                    for (int i = 0; i < num[0] + num[1]; ++i) {
                        if (position[2][i].x <= 10) {
                            int m = 2 * (total_num + 1) - 1;
                            message[m] = position[2][i].x;
                            int n = 2 * (total_num + 1);
                            message[n] = position[2][i].y;
                            total_num++;
                            // for (int j = 0; j < num[2]; ++j) { // x 轴
                            //     int n = 2 * (i + 1) - 1;
                            //     message[n] = position[2][i].x;
                            // }
                            // for (int j = 0; i < num[2]; ++i) { // y 轴
                            //     int n = 2 * (i + 1);
                            //     message[n] = position[2][i].y;
                            // }
                        }
                    }
                } else {
                    for (int i = 0; i < num[0] + num[1]; ++i) {
                        if (position[2][i].x > 10) {
                            int m = 2 * (total_num + 1) - 1;
                            message[m] = position[2][i].x - 10;
                            int n = 2 * (total_num + 1);
                            message[n] = position[2][i].y;
                            total_num++;
                        }
                    }
                }
                message[0] = total_num;
                // for (int i = 0; i < 2 * num + 1; ++i) {
                //     printf("%d ", message[i]);
                // }

                // 将位置数组打包成将要发送的字符串
                char message_str[512];
                for (int i = 0; i < 2 * message[0] + 1; ++i) {
                    sprintf(message_str + i * 3, "%02d ", message[i]);
                }
                sprintf(message_str + (2 * message[0] + 1) * 3, "\n");
                printf("%s", message_str);

                // 设置客户端地址
                client.sin_family = AF_INET;
                client.sin_addr.s_addr = inet_addr("192.168.43.130");
                client.sin_port = htons(PORT);
                if (car_num == 0) {
                    client.sin_addr.s_addr = inet_addr("192.168.43.130");
                } else {
                    client.sin_addr.s_addr = inet_addr("192.168.43.120");
                }

                // 向客户端发送信息
                int slen = sizeof(sockaddr_in);
                if (sendto(server_socket, message_str, strlen(message_str), 0, (sockaddr*)&client, sizeof(sockaddr_in)) == SOCKET_ERROR) {
                    printf("sendto() failed with error code: %d", WSAGetLastError());
                    return 3;
                }

                // 阻塞接收客户端的回复
                int message_len;
                if (message_len = recvfrom(server_socket, message, BUFLEN, 0, (sockaddr*)&client, &slen) == SOCKET_ERROR) {
                    printf("recvfrom() failed with error code: %d", WSAGetLastError());
                    exit(0);
                }

                // print details of the client/peer and the data received
                printf("Received packet from %s:%d\n", inet_ntoa(client.sin_addr), ntohs(client.sin_port));
                printf("Data: %s\n", message);

                // closesocket(server_socket);
                // WSACleanup();
            }
        } else {
            printf("quit");
            return 0;
        }
    }
}