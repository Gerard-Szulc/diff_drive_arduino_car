//
// Created by gerci on 09.12.2021.
//

#ifndef DIFFERENTIALSTEERING_H
#define DIFFERENTIALSTEERING_H

#include <Arduino.h>

struct DifferentialDriver {
    int leftMotor, rightMotor;
};

DifferentialDriver differentialSteering(int nJoyX, int nJoyY);

#endif //DIFFERENTIALSTEERING_H
