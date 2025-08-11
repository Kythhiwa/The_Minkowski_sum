#include "genran.h"


std::vector<std::pair<double, double>> Random::getRandPolygon(int n) {
    std::vector<std::pair<double, double>> polygon;
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-5.0, 5.0);
    std::uniform_real_distribution<double> radius_dist(1.0, 5.0);
    
    double center_x = dist(gen);
    double center_y = dist(gen);
    
    for(int i = 0; i < n; ++i) {
        double radius = radius_dist(gen);
        double angle = 2 * M_PI * i / n;  // Равномерное распределение углов
        
        // Добавляем случайное отклонение
        angle += dist(gen) * 0.1;
        
        double x = center_x + radius * cos(angle);
        double y = center_y + radius * sin(angle);
        
        polygon.emplace_back(x, y);
    }
    
    auto center = std::accumulate(polygon.begin(), polygon.end(), 
        std::make_pair(0.0, 0.0), [](auto sum, auto p) {
            return std::make_pair(sum.first + p.first, sum.second + p.second);
        });
    
    center.first /= polygon.size();
    center.second /= polygon.size();
    
    std::sort(polygon.begin(), polygon.end(), [center](auto& a, auto& b) {
        return atan2(a.second - center.second, a.first - center.first) < 
               atan2(b.second - center.second, b.first - center.first);
    });
    
    return polygon;
}
