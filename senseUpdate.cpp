#include "mcl.h"

void Robot::regularMove(const cv::Size& mapSize, float speed, float orient){
    orientation_ += orient * CV_PI / 180.0;
    position_.x += speed * std::cos(orientation_);
    position_.y += speed * std::sin(orientation_);  

    position_.x = std::max(0.0f, std::min((float)mapSize.width, position_.x));
    position_.y = std::max(0.0f, std::min((float)mapSize.height, position_.y));
    
    moveParticles(particles_, mapSize, speed, orient);
}
        
        
void Robot::CreateFov(){
    std::vector<cv::Point2f> triangle_points;
    
    triangle_points.clear();
    triangle_points.push_back(position_); // Sudut pertama
    triangle_points.push_back(cv::Point2f(position_.x + 200, position_.y - 180)); // Sudut kedua
    triangle_points.push_back(cv::Point2f(position_.x + 200, position_.y + 180)); // Sudut ketiga
    
    double angle = orientation_;
    // std::cout<<angle<<std::endl;
    
    // Buat mask segitiga
    rotated_.clear();
    rotated_.push_back(triangle_points[0]);
    rotated_.push_back(RotatePoint(triangle_points[1], triangle_points[0], angle));
    rotated_.push_back(RotatePoint(triangle_points[2], triangle_points[0], angle));
}

        

void Robot::LineScan(){
    // cv::Mat image = fieldmap_.clone(); 
    const cv::Mat& image = fieldmap_;
    if (image.empty()) {
        std::cerr << "Error: Image not loaded." << std::endl;
        return;
    }
    
    // Find contours of the objects in the image (assumes binary image)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat output = image.clone();
    std::vector<cv::Point2f> intersectionPoints, temp_points;
    
    cv::Point center = rotated_[0]; // Titik pusat robot
    cv::Point p1 = rotated_[1];
    cv::Point p2 = rotated_[2];
    
    int num_lines = 23;
    
    detected_.clear();
    distances_.clear();
    distance_.clear();
    
    for (int line_idx = 0; line_idx <= num_lines; ++line_idx) {
        float t = static_cast<float>(line_idx) / num_lines;  // Interpolasi antara sudut p1 dan p2
        cv::Point interpolated_point = p1 + t * (p2 - p1);
        
        cv::line(output, center, interpolated_point, cv::Scalar(255), 1);
        
        cv::LineIterator it(image, center, interpolated_point, 8);
        
        temp_points.clear();  // Untuk menyimpan intersection sementara dari garis ini
        
        for (int i = 0; i < it.count; i++, ++it) {
            if (image.at<uchar>(it.pos()) == 255) {  // Jika pixel putih (berpotongan dengan objek)
                cv::Point2f intersectionPoint(it.pos());
                temp_points.push_back(intersectionPoint);
                
                cv::circle(output, intersectionPoint, 3, cv::Scalar(255), 1);
            }
        }
        
        // Filter intersection points per line
        std::vector<cv::Point2f> filtered = FilterClose(temp_points);
        // distances_.push_back(getDistance(filtered));

        for(int i=0;i<filtered.size();i++){
            detected_.push_back(filtered[i]);
            distance_.push_back(GetDistance(filtered[i]));
        }
       
    }
    // DrawIntersectionPoints(output, intersectionPoints);
    // cv::imshow("Intersections", output);
}

//experiment

std::vector<std::vector<bool>> Robot::ConvertToBinaryMatrix(const cv::Mat& image) {
    if (image.empty()) {
        throw std::invalid_argument("Error: Image is empty.");
    }

    cv::Mat binaryImage;
    cv::threshold(image, binaryImage, 127, 255, cv::THRESH_BINARY);
    
    std::vector<std::vector<bool>> binaryMatrix(binaryImage.rows, std::vector<bool>(binaryImage.cols, false));
    for (int i = 0; i < binaryImage.rows; ++i) {
        for (int j = 0; j < binaryImage.cols; ++j) {
            binaryMatrix[i][j] = (binaryImage.at<uchar>(i, j) > 0);
        }
    }
    return binaryMatrix;
}

void Robot::LineScan(const std::vector<std::vector<bool>> binaryMatrix){
    try {
        // Konversi cv::Mat ke matriks biner (dilakukan di luar fungsi utama)
        // const std::vector<std::vector<bool>> binaryMatrix = ConvertToBinaryMatrix(src);

        // Variabel penting
        std::vector<cv::Point2f> temp_points;
        cv::Point center = rotated_[0]; // Titik pusat robot
        cv::Point p1 = rotated_[1];
        cv::Point p2 = rotated_[2];
        int num_lines = 23;

        detected_.clear();
        distances_.clear();
        distance_.clear();

        // Loop untuk menggambar garis
        for (int line_idx = 0; line_idx <= num_lines; ++line_idx) {
            float t = static_cast<float>(line_idx) / num_lines;  // Interpolasi antara sudut p1 dan p2
            cv::Point interpolated_point = p1 + t * (p2 - p1);

            // Iterasi melalui garis
            cv::LineIterator it(cv::Size(binaryMatrix[0].size(), binaryMatrix.size()), center, interpolated_point, 8);
            temp_points.clear();  // Untuk menyimpan intersection sementara dari garis ini

            for (int i = 0; i < it.count; ++i, ++it) {
                const cv::Point& pos = it.pos();
                if (binaryMatrix[pos.y][pos.x]) {  // Jika pixel adalah bagian dari objek
                    cv::Point2f intersectionPoint(pos);
                    temp_points.push_back(intersectionPoint);
                }
            }

            // Filter intersection points per line
            std::vector<cv::Point2f> filtered = FilterClose(temp_points);

            for (const auto& point : filtered) {
                detected_.push_back(point);
                distance_.push_back(GetDistance(point));
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}


// experiment 2 : landmark resampling
std::vector<Robot> Robot::resamplingLandmark(const std::vector<Robot>& particles) {
    // Early exit if no particles or no landmarks
    if (particles.empty() || landmarks_.empty() || lm_.empty()) {
        return {};
    }

    std::vector<Robot> newParticles;
    newParticles.reserve(particles.size());  // Pre-allocate memory

    // Create random number generators with thread_local for better performance
    thread_local static std::random_device rd;
    thread_local static std::mt19937 gen(rd());
    
    std::uniform_int_distribution<> angleDistribution(0, 359);
    std::normal_distribution<> noiseDistribution(0.0, 2.0);  // Centered noise distribution
    
    // Pre-filter matching landmarks to avoid repeated searching
    std::vector<Landmark> matchedLandmarks;
    for (const auto& landmark : landmarks_) {
        if (landmark.getID() == lm_[0].id && !landmark.positions.empty()) {
            matchedLandmarks.push_back(landmark);
        }
    }

    // If no matching landmarks, return empty
    if (matchedLandmarks.empty()) {
        return {};
    }

    // Shuffle matched landmarks for better randomness
    std::shuffle(matchedLandmarks.begin(), matchedLandmarks.end(), gen);

    // Efficient landmark sampling
    while (newParticles.size() < particles.size()) {
        for (const auto& landmark : matchedLandmarks) {
            // Randomly select a position from the landmark
            auto& landmarkPositions = landmark.positions;
            std::uniform_int_distribution<> posDistribution(0, landmarkPositions.size() - 1);
            
            Robot newParticle(fieldmap_);
            
            // Random angle and noise
            double randAngle = angleDistribution(gen);
            double distanceNoise = noiseDistribution(gen);

            // Update particle position
            cv::Point2f lmPos(landmarkPositions[posDistribution(gen)].first, 
                               landmarkPositions[posDistribution(gen)].second);
            
            Rotate(newParticle.position_.x, newParticle.position_.y, 
                   lm_[0].distance + distanceNoise, randAngle);
            newParticle.position_ += lmPos;
            
            newParticles.push_back(std::move(newParticle));

            // Break if we have enough particles
            if (newParticles.size() >= particles.size()) {
                return newParticles;
            }
        }
    }

    return newParticles;
}


std::vector<double> Robot::LineDistance(cv::Mat map){
    const cv::Mat& image = map;
    if (image.empty()) {
        std::cerr << "Error: Image not loaded." << std::endl;
        return std::vector<double>(); // Kembalikan vektor kosong
    }
    
    std::vector<double> dist;
    
    // Find contours of the objects in the image (assumes binary image)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat output = image.clone();
    std::vector<cv::Point2f> intersectionPoints, temp_points;
    
    cv::Point center = rotated_[0]; // Titik pusat robot
    cv::Point p1 = rotated_[1];
    cv::Point p2 = rotated_[2];

    // detected_.clear();
    
    int num_lines = 23;
    
    for (int line_idx = 0; line_idx <= num_lines; ++line_idx) {
        float t = static_cast<float>(line_idx) / num_lines;  // Interpolasi antara sudut p1 dan p2
        cv::Point interpolated_point = p1 + t * (p2 - p1);
        
        cv::line(output, center, interpolated_point, cv::Scalar(255), 1);
        
        cv::LineIterator it(image, center, interpolated_point, 8);
        
        temp_points.clear();  // Untuk menyimpan intersection sementara dari garis ini
        
        for (int i = 0; i < it.count; i++, ++it) {
            if (image.at<uchar>(it.pos()) == 255) {  // Jika pixel putih (berpotongan dengan objek)
                cv::Point2f intersectionPoint(it.pos());
                temp_points.push_back(intersectionPoint);
                
                cv::circle(output, intersectionPoint, 3, cv::Scalar(255), 1);
            }
        }
        
        // Filter intersection points per line
        std::vector<cv::Point2f> filtered = FilterClose(temp_points);

        for (int i = 0; i < filtered.size(); i++) {
            // detected_.push_back(filtered[i]);
            dist.push_back(GetDistance(filtered[i]));
        }
    }
    return dist;
}
