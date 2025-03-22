#include "mcl.h"

void control(float &move, float &orient){
    int key = cv::waitKey(1);  // Wait for a short period (1 ms) instead of blocking
    switch (key) {
        case 'w':
            orient = 0.0f;
            move = 2.0f;
            break;
        case 's':
            orient = 0.0f;
            move = -4.0f;
            break;
        case 'a':
            move = 0;
            orient = -1.0f;
            break;
        case 'd':
            move = 0;
            orient = 1.0f;
            break;
        default:
            move = 0;
            orient = 0;
            break;
    }
}

int Randomizer(int a, int b){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> value(a, b);
    
    return value(gen);
}

void drawMap(cv::Mat& map, const cv::Size& size){
    int width = size.width;
    int height = size.height;
    map = cv::Mat::zeros(height, width, CV_8UC1);
    
    cv::rectangle(map, cv::Point(50, 50), cv::Point(width - 50, height - 50), cv::Scalar(255, 255, 255), 2);

    cv::line(map, cv::Point(width / 2, 50), cv::Point(width / 2, height - 50), cv::Scalar(255), 2);

    cv::circle(map, cv::Point(width / 2, height / 2), 100, cv::Scalar(255), 2);

    cv::rectangle(map, cv::Point(50, 50 + 60), cv::Point(145, height - 50 - 60), cv::Scalar(255), 2);
    cv::rectangle(map, cv::Point(width - 50 - 95, 50 + 60), cv::Point(width - 50, height - 50 - 60), cv::Scalar(255), 2);
}

void on_trackbar(int, void* userdata){
    cv::Mat map;
    cv::Size size(950, 650);

    drawMap(map, size);
    
    cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);

    int* points = (int*)userdata;
    int x = points[0];
    int y = points[1];

    cv::circle(map, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

    cv::imshow("Triangle", map);
}

void debugLandmark(){
    //debugging purposes
    cv::namedWindow("Triangle");

    int points[2] = { 200, 200 };

    cv::createTrackbar("X", "Triangle", &points[0], 950, on_trackbar, points);
    cv::createTrackbar("Y", "Triangle", &points[1], 650, on_trackbar, points);

    on_trackbar(0, points);
}

void showSummary(std::vector<DetectedLandmark> lm){
    cv::Mat img = cv::Mat::zeros(300, 400, CV_8UC3);

    std::string jumlah = "Jumlah: " + std::to_string(lm.size());
    cv::putText(img, jumlah, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    for (int i = 0; i < lm.size(); i++) {
        std::string status = "(" + lm[i].id + ", " + std::to_string(lm[i].distance) + ")";
        
        int y_position = 60 + i * 30;
        
        if (y_position > img.rows - 10) break;

        cv::putText(img, status, cv::Point(10, y_position), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    }
    cv::imshow("control", img);
}

int main(){
    cv::Size mapSize(950, 650);
    cv::Mat map, display;
    int n = 2;
    
    cv::Point2f position = cv::Point2f(Randomizer(100, 800), Randomizer(100, 500));
    float orientation = (float) Randomizer(0, 359);
    float weight = 1.0f;

    drawMap(map, mapSize);

    Robot myRobot(map);
    myRobot.init(position, orientation, weight, n, mapSize);

    float updateMove = 0.0f, updateOrient = 0.0f;

    int control_value[2] = {1, 1};

    cv::namedWindow("control", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("move", "control", &control_value[0], 2);
    cv::createTrackbar("orient", "control", &control_value[1], 2);

    // debugLandmark();

    while (myRobot.MainLoop(control_value[0] + (-1), control_value[1] + (-1))){
        
        showSummary(myRobot.lm_);
        cv::imshow("Monte Carlo Localization Visualization", myRobot.display_);
        
        if (cv::waitKey(1)==27) break;
    }
    
    return 0;
}
