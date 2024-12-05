#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

#include "landmark.h"

const cv::Scalar merah = cv::Scalar(0, 0, 255);
const cv::Scalar hijau = cv::Scalar(0, 255, 0);
const cv::Scalar biru = cv::Scalar(255, 0, 0);

struct DetectedLandmark{
    std::string id;
    double distance;
};

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
        std::vector<cv::Point2f> detected_;
        std::vector<std::vector<double>> distances_;
        std::vector<double> distance_;

        std::vector<Landmark> landmarks_;
        std::vector<DetectedLandmark> lm_;

        std::vector<std::vector<bool>> BinaryMatrix_;

        // property
        Robot(cv::Mat mapInput);
        void init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize);
        std::vector<Robot> resampleParticles(const std::vector<Robot>& particles);
        bool MainLoop(float move, float orient);
        cv::Point2f getMeanPosition() const;
        void DetectingLandmark();

        //sense and update
        void regularMove(const cv::Size& mapSize, float speed, float orient);
        void CreateFov();
        void LineScan();

        //particles
        std::vector<Robot> initializeParticles(int numParticles, const cv::Size& mapSize);
        void moveParticles(std::vector<Robot>& particles, const cv::Size& mapSize, float& speed, float& orient);
        void activedParticleScan();

        //math and operations
        cv::Point RotatePoint(const cv::Point2f& p, const cv::Point2f& pivot, double angle);
        std::vector<cv::Point2f> FilterClose(std::vector<cv::Point2f> points);
        std::vector<double> getDistance(std::vector<cv::Point2f> points);
        double GetDistance(cv::Point2f points);
        double GetDistance(cv::Point points);
        double euclideanDistance(const std::vector<double>& v1, const std::vector<double>& v2);
        double calculateSimilarity(const std::vector<std::vector<double>>& distances1, const std::vector<std::vector<double>>& distances2);
        double calculateCosineSimilarity(const std::vector<double>& vec1, const std::vector<double>& vec2);
        double calculateCosineSimilarity(const std::vector<double>& vec1, const std::vector<double>& vec2, const std::vector<DetectedLandmark>& particleLm);
        double landmarkSimilarity(const std::vector<DetectedLandmark>& robotLm, const std::vector<DetectedLandmark>& particleLm);
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

        //experimental (di senseUpdate)
        std::vector<std::vector<bool>> ConvertToBinaryMatrix(const cv::Mat& image);
        void LineScan(const std::vector<std::vector<bool>> binaryMatrix);
        
};