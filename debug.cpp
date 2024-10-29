#include "mcl.h"


void Robot::drawRobot(cv::Mat& map) {
    cv::circle(map, position_, 5, hijau, -1);  // blue robotRobot
}

void Robot::drawParticles(cv::Mat& map, std::vector<Robot>& particles) {
     std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(particles.size());
    
    for (const auto& p : particles) {
        // Create a KeyPoint for each point (you can set size and angle as needed)
        keypoints.emplace_back(p.position_, 5); // '5' is an example size
    }
    
    // Draw all keypoints
    cv::drawKeypoints(map, keypoints, map, merah);
}


void Robot::printPoint(){
    for(const auto& p : distances_){
        for(const auto& pc : p){
            std::cout<<pc;
        }
        std::cout<<std::endl;
    }
    std::cout<<distances_.size()<<std::endl;
}

void Robot::printSimilarity(){
    double sim;
    for(int i=0;i<particles_.size();i++){
        sim = calculateSimilarity(distances_, particles_[i].distances_);
        std::cout<<sim<<" "<<i<<std::endl;
    }
    std::cout<<particles_.size()<<std::endl;
}

void Robot::LineScan(int i){
    cv::Mat image = fieldmap_.clone();    
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
    
    int num_lines = 15;
    
    detected_.clear();
    distances_.clear();
    
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
        distances_.push_back(getDistance(filtered));
        detected_.push_back(filtered);
    }
    // DrawIntersectionPoints(output, intersectionPoints);
    cv::imshow(std::to_string(i), output);
}

void Robot::DrawIntersectionPoints(cv::Mat& image, const std::vector<cv::Point2f>& intersectionPoints) {
    // Buat salinan dari image asli untuk digambar
    cv::Mat output = image.clone();
    output = cv::Mat::zeros(image.size(), image.type());
    
    // Pastikan image dalam format BGR, jika tidak ubah dari grayscale ke BGR
    if (output.channels() == 1) {
        cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
    }
    
    // Tentukan warna dan ukuran titik
    cv::Scalar pointColor(0, 0, 255);  // Warna merah untuk titik (BGR)
    int radius = 3;                    // Radius dari titik
    int thickness = -1;                // -1 untuk menggambar lingkaran penuh
    
    // Loop melalui semua intersection points dan gambar pada image
    for (const auto& point : intersectionPoints) {
        cv::circle(output, point, radius, pointColor, thickness);
    }
    
    cv::imshow("Intersection Points", output);
}

void Robot::drawFov(cv::Mat& field){
    if(!field.empty()){
        cv::polylines(field, rotated_, true, cv::Scalar(255, 0, 255), 1);
    }   
}