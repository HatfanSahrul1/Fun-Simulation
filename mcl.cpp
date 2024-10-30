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

int main(){
    cv::Size mapSize(950, 650);
    cv::Mat map, display;
    int n = 400;
    
    cv::Point2f position = cv::Point2f(mapSize.width / 2 + 125, mapSize.height / 2 - 90);
    float orientation = -0.5f;
    float weight = 1.0f;

    drawMap(map, mapSize);

    Robot myRobot(map);
    myRobot.init(position, orientation, weight, n, mapSize);

    float updateMove = 0.0f, updateOrient = 0.0f;




    while (myRobot.MainLoop(updateMove, updateOrient)){
        
        cv::imshow("Monte Carlo Localization Visualization", myRobot.display_);

        updateMove=0;
        updateOrient=0;
        control(updateMove, updateOrient);
        
        if (cv::waitKey(1)==27) break;
    }
    
    return 0;
}
