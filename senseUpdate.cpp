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