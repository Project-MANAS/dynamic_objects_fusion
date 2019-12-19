//
// Created by shivesh on 12/16/19.
//

#include "dynamic_objects_fusion/sensor_object.hpp"

namespace dynamic_objects_fusion {

SensorObject::SensorObject() {

}

SensorObject::~SensorObject() {

}

SensorObject::SensorObject(const std::shared_ptr<SensorObject> sensor_object) {

}

SensorObjects::SensorObjects() {

}

SensorObjects::SensorObjects(int id, double timestamp) {
  sensor_id_ = id;
  timestamp_ = timestamp;
}

SensorObjects::~SensorObjects() {

}
}
