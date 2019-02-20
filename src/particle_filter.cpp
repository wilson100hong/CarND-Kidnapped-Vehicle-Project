
/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::sin;
using std::cos;

#define EPS 1e-6


double MultivProb(double x, double y, double m_x, double m_y, double std_x, double std_y) {
  double fx = (x-m_x)*(x-m_x) / (2*std_x*std_x);
  double fy = (y-m_y)*(y-m_y) / (2*std_y*std_y);
  return exp(-(fx + fy)) / (2*M_PI*std_x*std_y);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 1000;

  particles.clear();
  particles.reserve(num_particles);
  weights.clear();
  weights.reserve(num_particles);

  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i=0;i<num_particles;++i) {
    double sample_x, sample_y, sample_theta;
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);
    particles.push_back({i, sample_x, sample_y, sample_theta,
                         /*weight=*/1.0, /*associations=*/{},
                         /*sense_x=*/{}, /*sense_y=*/{}});
    weights.push_back(1.0);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  std::cout << "prediction" << std::endl;
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  double yrdt = yaw_rate * delta_t;
  double vdt = velocity * delta_t;

  for (auto& p : particles) {
    double pred_x, pred_y;
    // Use CVTR model. Note that needs to handle zero yaw rate differently.
    if (fabs(yaw_rate) > EPS) {
       double v_yr = velocity / yaw_rate;
       pred_x = p.x + v_yr*(sin(p.theta + yrdt) - sin(p.theta)); 
       pred_y = p.y + v_yr*(cos(p.theta) - cos(p.theta + yrdt));
    } else {
       pred_x = p.x + vdt*cos(p.theta);
       pred_y = p.y + vdt*sin(p.theta);
    }
    double pred_theta = p.theta + yrdt;

    normal_distribution<double> dist_x(pred_x, std_pos[0]);
    normal_distribution<double> dist_y(pred_y, std_pos[1]);
    normal_distribution<double> dist_theta(pred_theta, std_pos[2]);

    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);

    p.associations.clear();
    p.sense_x.clear();
    p.sense_y.clear();
    p.weight = 1;
  }
}

void ParticleFilter::dataAssociation(const vector<LandmarkObs>& predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   */
  for (LandmarkObs& ob : observations) {
    double min_dist = std::numeric_limits<double>::max();
    for (const LandmarkObs& pred : predicted) {
      double dist = std::hypot(ob.x - pred.x, ob.y - pred.y);
      if (dist < min_dist) {
        min_dist = dist;
        ob.id = pred.id;
      }
    } 
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs>& observations, 
                                   const Map& map_landmarks) {
  std::cout << "updateWeights" << std::endl;
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // Build mappintg from landmark id to landmark.
  std::map<int, Map::single_landmark_s> id_landmarks;
  for (const auto& landmark : map_landmarks.landmark_list) {
    id_landmarks[landmark.id_i] = landmark;
  }

  int index = 0;
  for (auto& p : particles) {
    vector<LandmarkObs> transformed;
    transformed.reserve(observations.size());
    // Transform observation from VEHICLE'S coordinate to MAP'S coordinate.
    for (const auto& o : observations) {
      double m_x = o.x*cos(p.theta) - o.y*sin(p.theta) + p.x;
      double m_y = o.x*sin(p.theta) + o.y*cos(p.theta) + p.y;
      transformed.push_back({-1, m_x, m_y});
    }
    // Build predicted observation from map. Ignore landmarks out of sensor range.
    vector<LandmarkObs> predicted;
    predicted.reserve(map_landmarks.landmark_list.size());
    for (const auto& landmark : map_landmarks.landmark_list) {
      double dist = std::hypot(landmark.x_f - p.x, landmark.y_f - p.y);
      if (dist <= sensor_range) {
        predicted.push_back({landmark.id_i, landmark.x_f, landmark.y_f});
      }
    }
    // Associate transformed landmarks to map.
    dataAssociation(predicted, transformed);

    // Update particle associations, sense_x, sense_y and weight
    p.weight = 1.0;
    p.associations.reserve(transformed.size());
    p.sense_x.reserve(transformed.size());
    p.sense_y.reserve(transformed.size());
    for (const auto& t : transformed) {
      // Skip observation which has no association
      if (t.id >= 0) {
        p.associations.push_back(t.id);
        p.sense_x.push_back(t.x);
        p.sense_y.push_back(t.y);
        const auto& landmark = id_landmarks[t.id];
        double prob = MultivProb(
            t.x, t.y, landmark.x_f, landmark.y_f, std_landmark[0], std_landmark[1]);
        p.weight *= prob;
      }
    }
    weights[index++] = p.weight;
  }
}

void ParticleFilter::resample() {
  std::cout << "resample" << std::endl;
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<> d(weights.begin(), weights.end());
  std::vector<Particle> resampled;
  resampled.reserve(num_particles);
  for (int i=0;i<num_particles;++i) {
    int p_i = d(gen);  // particle index
    resampled.push_back(particles[p_i]);
  }
  particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
