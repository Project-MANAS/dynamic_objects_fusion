//
// Created by shivesh on 12/16/19.
//

#include <cmath>
#include <queue>
#include "dynamic_objects_fusion/dynamic_objects_fusion.hpp"

namespace dynamic_objects_fusion {

DynamicObjectsFusion::DynamicObjectsFusion() {

}

DynamicObjectsFusion::~DynamicObjectsFusion() {

}

void DynamicObjectsFusion::fuse(const std::shared_ptr<SensorObjects> sensor_objects) {
  if (sensor_objects->objects_.empty()) {
    return;
  }
  auto unassociated_sensor_objects = std::make_shared<std::vector<int>>();
  auto unassociated_fused_objects = std::make_shared<std::vector<int>>();
  auto association = associate_old_objects(sensor_objects, unassociated_sensor_objects, unassociated_fused_objects);
  auto association_matrix = compute_association_matrix(
      sensor_objects, unassociated_sensor_objects, unassociated_fused_objects);
  associate_new_objects(sensor_objects, association_matrix, association, unassociated_sensor_objects, unassociated_fused_objects);
  update_associated_fused_objects(sensor_objects, association);
  update_unassociated_fused_objects(unassociated_fused_objects, sensor_objects->timestamp_);
  create_new_fused_objects(sensor_objects, unassociated_sensor_objects);

  delete_expired_fused_objects(sensor_objects->timestamp_);
  send_fused_objects();
}

std::shared_ptr<std::unordered_map<int,int>> DynamicObjectsFusion::associate_old_objects(
    std::shared_ptr<SensorObjects> sensor_objects,
    std::shared_ptr<std::vector<int>> unassociated_sensor_objects,
    std::shared_ptr<std::vector<int>> unassociated_fused_objects) {
  auto association = std::make_shared<std::unordered_map<int, int>>();
  std::unordered_map<int, int> fused_obj_id_to_sensor_obj_id;
  std::unordered_set<int> associated_fused_objects;

  for (int i = 0; i < fused_objects_.objects_.size(); ++i) {
    auto fused_object = fused_objects_.objects_[i];
    fused_obj_id_to_sensor_obj_id[fused_object->sensor_object_id_[sensor_objects->sensor_id_]] = i;
  }
  for (int i = 0; i < sensor_objects->objects_.size(); ++i) {
    auto sensor_object = sensor_objects->objects_[i];
    auto it = fused_obj_id_to_sensor_obj_id.find(sensor_object->id_);
    if (it != fused_obj_id_to_sensor_obj_id.end()) {
      association->insert({it->second, i});
      associated_fused_objects.insert(it->second);
    } else {
      unassociated_sensor_objects->push_back(i);
    }
  }
  for (int i = 0; i < fused_objects_.objects_.size(); ++i) {
    auto fused_object = fused_objects_.objects_[i];
    if (associated_fused_objects.find(i) == associated_fused_objects.end()) {
      unassociated_fused_objects->push_back(i);
    }
  }
  return association;
}

std::shared_ptr<std::vector<std::vector<double>>> DynamicObjectsFusion::compute_association_matrix(
    std::shared_ptr<SensorObjects> sensor_objects,
    std::shared_ptr<std::vector<int>> unassociated_sensor_objects,
    std::shared_ptr<std::vector<int>> unassociated_fused_objects) {

  auto association_matrix = std::make_shared<std::vector<std::vector<double>>>();
  association_matrix->resize(unassociated_fused_objects->size());

  for (int i = 0; i < unassociated_fused_objects->size(); ++i) {
    (*association_matrix)[i].resize(unassociated_sensor_objects->size());
    for (int j = 0; j < unassociated_sensor_objects->size(); ++j) {
      (*association_matrix)[i][j] = compute_distance(
          fused_objects_.objects_[(*unassociated_fused_objects)[i]],
          sensor_objects->objects_[(*unassociated_sensor_objects)[j]]);
    }
  }

  return association_matrix;
}

double DynamicObjectsFusion::compute_distance(std::shared_ptr<FusedObject> fused_object,
                                              std::shared_ptr<SensorObject> sensor_object) {
  return hypot(fused_object->position_[0] - sensor_object->position_[0],
      fused_object->position_[1] - sensor_object->position_[1]);
}

void DynamicObjectsFusion::associate_new_objects(
    std::shared_ptr<SensorObjects> sensor_objects,
    std::shared_ptr<std::vector<std::vector<double>>> association_matrix,
    std::shared_ptr<std::unordered_map<int, int>> association,
    std::shared_ptr<std::vector<int>> unassociated_sensor_objects,
    std::shared_ptr<std::vector<int>> unassociated_fused_objects) {

  double max_dist = 2.5;

  std::vector<std::vector<int>> nb_graph(unassociated_fused_objects->size() + unassociated_sensor_objects->size());
  for (int i = 0; i < unassociated_fused_objects->size(); i++) {
    for (int j = 0; j < unassociated_sensor_objects->size(); j++) {
      if ((*association_matrix)[i][j] < max_dist) {
        nb_graph[i].push_back(unassociated_fused_objects->size() + j);
        nb_graph[j + unassociated_fused_objects->size()].push_back(i);
      }
    }
  }
  std::vector<std::pair<std::vector<int>, std::vector<int>>> components;

  std::vector<bool> visited(nb_graph.size(), false);
  std::queue<int> que;

  for (int i = 0; i < nb_graph.size(); ++i) {
    if (visited[i]) {
      continue;
    }
    std::pair<std::vector<int>, std::vector<int>> component;
    if (i < unassociated_fused_objects->size()) {
      component.first.push_back(i);
    } else {
      component.second.push_back(i - unassociated_fused_objects->size());
    }

    que.push(i);
    visited[i] = true;
    while (!que.empty()) {
      int id = que.front();
      que.pop();
      for (int nb_id : nb_graph[id]) {
        if (visited[nb_id] == 0) {
          if (nb_id < unassociated_fused_objects->size()) {
            component.first.push_back(nb_id);
          } else {
            component.second.push_back(nb_id - unassociated_fused_objects->size());
          }
          que.push(nb_id);
          visited[nb_id] = true;
        }
      }
    }
    components.push_back(component);
  }

  for (const auto& component : components) {
    if (component.first.empty() || component.second.empty()) {
      continue;
    } else if (component.first.size() == 1 && component.second.size() == 1) {
      if ((*association_matrix)[component.first[0]][component.second[0]] < max_dist) {
        association->insert({(*unassociated_fused_objects)[component.first[0]],
                             (*unassociated_sensor_objects)[component.second[0]]});
        fused_objects_.objects_[(*unassociated_fused_objects)[component.first[0]]]->sensor_object_id_[sensor_objects->sensor_id_] =
            sensor_objects->objects_[(*unassociated_sensor_objects)[component.second[0]]]->id_;
        (*unassociated_fused_objects)[component.first[0]] = -1;
        (*unassociated_sensor_objects)[component.second[0]] = -1;
      }
      continue;
    }
    // compute local cost matrix and idx mapping
    std::vector<std::vector<double>> local_association_matrix;
    std::vector<int> fusion_l2g;
    std::vector<int> sensor_l2g;
    local_association_matrix.resize(component.first.size());
    for (size_t j = 0; j < component.first.size(); ++j) {
      fusion_l2g.push_back(component.first[j]);

      for (size_t k = 0; k < component.second.size(); ++k) {
        if (j == 0) {
          sensor_l2g.push_back(component.second[k]);
        }
        local_association_matrix[j].push_back((*association_matrix)[component.first[j]][component.second[k]]);
      }
    }

    std::vector<int> fusion_idxs, sensor_idxs;
    if (!local_association_matrix.empty() && !local_association_matrix[0].empty()) {
      HungarianOptimizer hungarian_optimizer(local_association_matrix);
      hungarian_optimizer.minimize(&fusion_idxs, &sensor_idxs);
    }

    for (size_t j = 0; j < fusion_idxs.size(); ++j) {
      int f_idx = fusion_idxs[j];
      int s_idx = sensor_idxs[j];
      if (local_association_matrix[f_idx][s_idx] < max_dist) {
       association->insert({(*unassociated_fused_objects)[fusion_l2g[f_idx]], (*unassociated_sensor_objects)[sensor_l2g[s_idx]]});
        fused_objects_.objects_[(*unassociated_fused_objects)[fusion_l2g[f_idx]]]->sensor_object_id_[sensor_objects->sensor_id_] =
            sensor_objects->objects_[(*unassociated_sensor_objects)[sensor_l2g[s_idx]]]->id_;
        (*unassociated_fused_objects)[fusion_l2g[f_idx]] = -1;
        (*unassociated_sensor_objects)[sensor_l2g[s_idx]] = -1;
      }
    }
  }
  int unassociated_fused_object_num = 0;
  for (size_t i = 0; i < unassociated_fused_objects->size(); ++i) {
    if ((*unassociated_fused_objects)[i] >= 0) {
      (*unassociated_fused_objects)[unassociated_fused_object_num++] = (*unassociated_fused_objects)[i];
    }
  }
  unassociated_fused_objects->resize(unassociated_fused_object_num);
  int unassociated_sensor_object_num = 0;
  for (size_t i = 0; i < unassociated_sensor_objects->size(); ++i) {
    if ((*unassociated_sensor_objects)[i] >= 0) {
      (*unassociated_sensor_objects)[unassociated_sensor_object_num++] = (*unassociated_sensor_objects)[i];
    }
  }
  unassociated_sensor_objects->resize(unassociated_sensor_object_num);
}

void DynamicObjectsFusion::update_associated_fused_objects(std::shared_ptr<SensorObjects> sensor_objects,
    std::shared_ptr<std::unordered_map<int,int>> association) {
  for (auto assignment : *association) {
    fused_objects_.objects_[assignment.first]->update(sensor_objects->objects_[assignment.second], sensor_objects->timestamp_);
  }
}
void DynamicObjectsFusion::update_unassociated_fused_objects(std::shared_ptr<std::vector<int>> unassociated_fused_objects, double timestamp) {
  for (auto idx : *unassociated_fused_objects) {
    fused_objects_.objects_[idx]->update(timestamp);
  }
}

void DynamicObjectsFusion::create_new_fused_objects(std::shared_ptr<SensorObjects> sensor_objects,
  std::shared_ptr<std::vector<int>> unassociated_sensor_objects) {
  for (auto idx : *unassociated_sensor_objects) {
    auto sensor_object = sensor_objects->objects_[idx];
    auto fused_object = std::make_shared<FusedObject>(sensor_object, sensor_objects->timestamp_, sensor_objects->sensor_id_);
    fused_objects_.objects_.push_back(fused_object);
  }
}

void DynamicObjectsFusion::delete_expired_fused_objects(double timestamp) {
  for (auto iter = fused_objects_.objects_.begin(); iter != fused_objects_.objects_.end(); ++iter) {
    if ((timestamp - (*iter)->timestamp_) > 1) {
      fused_objects_.objects_.erase(iter--);
    }
  }
}

void DynamicObjectsFusion::send_fused_objects() {

}
}