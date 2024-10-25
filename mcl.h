#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

const cv::Scalar merah = cv::Scalar(0, 0, 255);
const cv::Scalar hijau = cv::Scalar(0, 255, 0);


class Robot
{
    private:
    public:
        int n_particles_;
        cv::Point2f position_;
        float orientation_;
        float weight_;
        std::vector<Robot> particles_;
        cv::Size mapSize_;
        cv::Mat fov, fieldmap_, inputPoint;

        std::vector<cv::Point> rotated_;
        std::vector<std::vector<cv::Point2f>> detected_;
        std::vector<std::vector<double>> distances_;

        // property
        Robot(cv::Mat mapInput);
        void init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize);

        //sense and update
        void regularMove(Robot& robot, const cv::Size& mapSize, float speed, float orient);
        void CreateFov(cv::Mat field);
        void LineScan();
        void DrawIntersectionPoints(cv::Mat& image, const std::vector<cv::Point2f>& intersectionPoints);

        //particles
        std::vector<Robot> initializeParticles(int numParticles, const cv::Size& mapSize);
        void moveParticles(std::vector<Robot>& particles, const cv::Size& mapSize, float& speed, float& orient);
        void activedParticleScan();

        //math and operations
        cv::Point RotatePoint(const cv::Point2f& p, const cv::Point2f& pivot, double angle);
        std::vector<cv::Point2f> FilterClose(std::vector<cv::Point2f> points);
        std::vector<double> getDistance(std::vector<cv::Point2f> points);
        double euclideanDistance(const std::vector<double>& v1, const std::vector<double>& v2);
        double calculateSimilarity(const std::vector<std::vector<double>>& distances1, const std::vector<std::vector<double>>& distances2);

        //debug and visualization
        void drawRobot(cv::Mat& map);
        void drawParticles(cv::Mat& map, std::vector<Robot>& particles);
        void printPoint();
        void printSimilarity();
        void LineScan(int i);

        
};