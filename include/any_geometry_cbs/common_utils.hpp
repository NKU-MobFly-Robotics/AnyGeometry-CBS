#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include "environment.hpp"

namespace libMultiRobotPlanning {

inline int turnLeft(int o) {
    return (o == 1) ? 4 : o - 1;
}

inline int turnRight(int o) {
    return (o == 4) ? 1 : o + 1;
}

inline std::vector<std::vector<std::vector<Location>>> computeShapeSheet(
    const std::vector<std::vector<Location>>& shapes, size_t agentNum) {
    
    std::vector<std::vector<std::vector<Location>>> shapesSheet(
        agentNum, std::vector<std::vector<Location>>(5));
    
    for (size_t i = 0; i < agentNum; i++) {
        for (int o = 1; o <= 4; o++) {
            std::vector<Location> shapeTemp;
            Location s(0, 0);
            
            switch (o) {
                case 1:
                    shapeTemp = shapes[i];
                    break;
                case 2:
                    for (const auto& pt : shapes[i]) {
                        s.x = -pt.y;
                        s.y = pt.x;
                        shapeTemp.emplace_back(s);
                    }
                    break;
                case 3:
                    for (const auto& pt : shapes[i]) {
                        s.x = -pt.x;
                        s.y = -pt.y;
                        shapeTemp.emplace_back(s);
                    }
                    break;
                case 4:
                    for (const auto& pt : shapes[i]) {
                        s.x = pt.y;
                        s.y = -pt.x;
                        shapeTemp.emplace_back(s);
                    }
                    break;
                default:
                    shapeTemp = shapes[i];
                    break;
            }
            shapesSheet[i][o] = shapeTemp;
        }
    }
    return shapesSheet;
}

inline std::vector<int> computeTouchSpotsList(
    const std::vector<std::vector<std::vector<Location>>>& shapesSheet) {
    
    std::vector<int> h1List;
    
    for (size_t i = 0; i < shapesSheet.size(); i++) {
        std::vector<Eigen::Vector3d> tempList;
        
        for (size_t j = 0; j < shapesSheet[i][1].size(); j++) {
            for (int k = 1; k < 5; k++) {
                Eigen::Vector3d touchTemp;
                touchTemp(0) = shapesSheet[i][1][j].x * 1.0 + 
                               0.5 * (sin(1.5 * k) > 0 ? 1 : -1);
                touchTemp(1) = shapesSheet[i][1][j].y * 1.0 + 
                               0.5 * (cos(1.5 * k) > 0 ? 1 : -1);
                touchTemp(2) = 0.0;
                
                bool found = false;
                for (auto it = tempList.begin(); it != tempList.end(); ++it) {
                    if (it->isApprox(touchTemp)) {
                        tempList.erase(it);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    tempList.push_back(touchTemp);
                }
            }
        }
        h1List.push_back(tempList.size());
    }
    return h1List;
}

}  // namespace libMultiRobotPlanning
