#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <cmath>

struct Particle {
    cv::Point2f position;
    float orientation;
    float weight;
};

// Initialize random particles
std::vector<Particle> initializeParticles(int numParticles, const cv::Size& mapSize) {
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disX(0, mapSize.width);
    std::uniform_real_distribution<> disY(0, mapSize.height);
    std::uniform_real_distribution<> disTheta(0, 2 * CV_PI);
    
    for (int i = 0; i < numParticles; ++i) {
        Particle p;
        p.position = cv::Point2f(disX(gen), disY(gen));
        p.orientation = disTheta(gen);
        p.weight = 1.0 / numParticles;  // uniform weight
        particles.push_back(p);
    }
    return particles;
}

// Visualize particles
void drawParticles(cv::Mat& map, const std::vector<Particle>& particles) {
    for (const auto& p : particles) {
        cv::circle(map, p.position, 2, cv::Scalar(0, 0, 255), -1);  // red particles
    }
}

// Move particles randomly
void moveParticles(std::vector<Particle>& particles, const cv::Size& mapSize, float& speed, float& orient) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> moveDist(0, 5);  // random move
    std::normal_distribution<float> turnDist(0, 0.1); // random orientation change
    
    for (auto& p : particles) {

        p.orientation += orient;  // random orientation change
        p.position.x += speed * std::cos(p.orientation);
        p.position.y += speed * std::sin(p.orientation);
        
        // Keep particles within map bounds
        p.position.x = std::max(0.0f, std::min((float)mapSize.width, p.position.x));
        p.position.y = std::max(0.0f, std::min((float)mapSize.height, p.position.y));
    }
}

// Draw robot (true position)
void drawRobot(cv::Mat& map, const cv::Point2f& position) {
    cv::circle(map, position, 5, cv::Scalar(255, 0, 0), -1);  // blue robot
}

void regularMove(Particle& robot, std::vector<Particle>& particles, const cv::Size& mapSize, float speed, float orient){
    robot.orientation += orient;
    robot.position.x += speed * std::cos(robot.orientation);
    robot.position.y += speed * std::sin(robot.orientation);

    robot.position.x = std::max(0.0f, std::min((float)mapSize.width, robot.position.x));
    robot.position.y = std::max(0.0f, std::min((float)mapSize.height, robot.position.y));

    moveParticles(particles, mapSize, speed, orient);
}

void control(float &move, float &orient){
    if(cv::waitKey(0)=='w'){
        move = 2.0f;
    }else if(cv::waitKey(0)=='s'){
        move = -2.0f;
    }else if(cv::waitKey(0)=='a'){
        orient = -.5f;
    }else if(cv::waitKey(0)=='d'){
        orient = .5f;   
    }else{
        move = 0;
        orient = 0;
    }
}

int main() {
    // Create a blank map
    cv::Size mapSize(900, 600);
    cv::Mat map;

    // Initialize particles
    int numParticles = 100;
    std::vector<Particle> particles = initializeParticles(numParticles, mapSize);
    
    Particle robot;
    // Robot's true position
    robot.position = cv::Point2f(mapSize.width / 2, mapSize.height / 2);
    robot.orientation = 0.0f;
    robot.weight = 1.0f;
    
    
    int robotState = 0; // 0: forward, 1: stop, 2: rotate
    int frameCount = 0;
    
    float move = 0.0f, orient = 0.0f;

    while (true) {
        map = cv::Mat::zeros(mapSize, CV_8UC3);

        drawParticles(map, particles);
        
        drawRobot(map, robot.position);


        regularMove(robot, particles, mapSize, move, orient);
        cv::imshow("Monte Carlo Localization Visualization", map);
        move=0;
        orient=0;
        control(move, orient);
        
        if (cv::waitKey(0)==27) break;
    }
    
    return 0;
}
