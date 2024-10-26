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

    std::vector<cv::Point2f> l = {cv::Point2f(width*2/3, 50), cv::Point2f(width*2/3, height * 3/8), cv::Point2f((width*2/3) + 70, height * 3/8)};
    std::vector<cv::Point2f> t = {cv::Point2f(width*3/5, height*2/3), cv::Point2f(width-50, height*2/3), 
                                    cv::Point2f(width*3/5, (height*2/3)-25), cv::Point2f(width*3/5, (height*2/3)+25)};
    std::vector<cv::Point2f> i = {cv::Point2f(width*4/10, 120), cv::Point2f(width*4/10, height-50)};

    // l
    cv::line(map, l[0], l[1], cv::Scalar(255), 2);
    cv::line(map, l[1], l[2], cv::Scalar(255), 2);

    // t 
    cv::line(map, t[0], t[1], cv::Scalar(255), 2);
    cv::line(map, t[2], t[3], cv::Scalar(255), 2);

    // i
    cv::line(map, i[0], i[1], cv::Scalar(255), 2);
}

int main(){
    cv::Size mapSize(950, 650);
    cv::Mat map, display;
    int n = 200;
    
    cv::Point2f position = cv::Point2f(mapSize.width / 2, mapSize.height / 2);
    float orientation = 20.0f;
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