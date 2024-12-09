#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

class Landmark {
    private:
        std::string id; 

    public:
        std::vector<std::pair<int, int>> positions;
        
        Landmark(const std::string& landmark_id, const std::vector<std::pair<int, int>>& pos_list)
            : id(landmark_id), positions(pos_list) {}

        bool isDetected(std::vector<cv::Point>& area, std::pair<int, int>& pos){
            return cv::pointPolygonTest(area, cv::Point2f(pos.first, pos.second), false) >= 0;
        }

        std::string getID() const {
            return id;
        }
};