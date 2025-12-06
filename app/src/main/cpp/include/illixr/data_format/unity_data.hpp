#pragma once

namespace ILLIXR::data_format {

struct unity_pose {
    float positionX;
    float positionY;
    float positionZ;
    float quatX;
    float quatY;
    float quatZ;
    float quatW;
    int isTracked;
};

}