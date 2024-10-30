#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
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
        double weight_;
        std::vector<Robot> particles_;
        cv::Size mapSize_;
        cv::Mat fieldmap_, display_;

        std::vector<cv::Point> rotated_;
        std::vector<std::vector<cv::Point2f>> detected_;
        std::vector<std::vector<double>> distances_;
        std::vector<double> distance_;

        // property
        Robot(cv::Mat mapInput);
        void init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize);
        std::vector<Robot> resampleParticles(const std::vector<Robot>& particles);
        bool MainLoop(float move, float orient);
        cv::Point2f getMeanPosition() const;

        //sense and update
        void regularMove(const cv::Size& mapSize, float speed, float orient);
        void CreateFov();
        void LineScan();

        //particles
        std::vector<Robot> initializeParticles(int numParticles, const cv::Size& mapSize);
        void moveParticles(std::vector<Robot>& particles, const cv::Size& mapSize, float& speed, float& orient);
        std::vector<Robot> resetWeights(std::vector<Robot>& particles);
        std::vector<Robot> resetParticles(std::vector<Robot> particles);
        void activedParticleScan();

        //math and operations
        cv::Point RotatePoint(const cv::Point2f& p, const cv::Point2f& pivot, double angle);
        std::vector<cv::Point2f> FilterClose(std::vector<cv::Point2f> points);
        std::vector<double> getDistance(std::vector<cv::Point2f> points);
        double GetDistance(cv::Point2f points);
        double euclideanDistance(const std::vector<double>& v1, const std::vector<double>& v2);
        double calculateSimilarity(const std::vector<std::vector<double>>& distances1, const std::vector<std::vector<double>>& distances2);
        double calculateCosineSimilarity(const std::vector<double>& vec1, const std::vector<double>& vec2);
        void normalizeWeights(std::vector<Robot>& particles);
        double getMaxWeight(const std::vector<Robot>& particles);
        double averageWeight(const std::vector<Robot>& robots);

        //debug and visualization
        void drawRobot(cv::Mat& map);
        void drawFov(cv::Mat& field);
        void drawParticles(cv::Mat& map, std::vector<Robot>& particles);
        void DrawIntersectionPoints(cv::Mat& image, const std::vector<cv::Point2f>& intersectionPoints);
        void printPoint();
        void printSimilarity();
        void LineScan(int i);

        
};