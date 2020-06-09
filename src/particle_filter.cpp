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
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <stdio.h>
#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution ;
void ParticleFilter::init(double x, double y, double theta, double std[]) {
    //define random engine
    std::default_random_engine gen ;
    num_particles = 100;  // TODO: Set the number of particles
    //initialize all the particles with gps location define normal distributions
    normal_distribution<double> normal_x(x , std[0]);
    normal_distribution<double> normal_y(y , std[1]);
    normal_distribution<double> normal_theta( theta , std[2]);
    
    //generate the number of particles
    for (int i =0 ; i<num_particles ; i++){
      //initialize a singla particle
      Particle particle ;
      particle.id = i ;
      particle.x = normal_x(gen);
      particle.y = normal_y(gen);
      particle.theta = normal_theta(gen);
      particle.weight = 1.0 ;
  
      //add this particle to the patricle list
      particles.push_back(particle);
    }
    is_initialized = true;
    std::cout <<"Working"<<std::endl;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
    //define a random generator
    std::cout << "Predictioin setp"<<std::endl;
    std::default_random_engine gen ;
    //define the random states
    normal_distribution<double> normal_x(0.0 , std_pos[0]);
    normal_distribution<double> normal_y(0.0 , std_pos[1]);
    normal_distribution<double> normal_theta(0.0 , std_pos[2]);

    for (int i =0 ; i< num_particles;i++){
      if(fabs(yaw_rate) > 0.00001){

        //update the x and y motion
       
        particles[i].x += (velocity /yaw_rate ) * (  sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta) ) + normal_x(gen);
        particles[i].y += (velocity / yaw_rate ) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)) + normal_y(gen);
        particles[i].theta += yaw_rate*delta_t + normal_theta(gen);
      }

      else{
        particles[i].x = particles[i].x + velocity * cos ( particles[i].theta)*delta_t + normal_x(gen);
        particles[i].y = particles[i].y + velocity * sin(particles[i].theta)*delta_t + normal_y(gen);
        particles[i].theta = particles[i].theta + normal_theta(gen);
      }


      double pi_2 = 2*M_PI;
      if(particles[i].theta > 0){
        if(particles[i].theta > pi_2){
          particles[i].theta = particles[i].theta - pi_2;
        }
      }
      else{
        if(particles[i].theta < -pi_2){
          particles[i].theta = particles[i].theta + pi_2;
        }
      }
    }


}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
    for (int i =0 ; i<observations.size() ; i ++){
      double distance = std::numeric_limits<double>::infinity();
      int index ;
      for(int j =0 ; j < predicted.size() ; j++){
          double dist = sqrt (  pow(observations[i].x - predicted[j].x , 2 )+pow( observations[i].y - predicted[j].y , 2 ));
          if(dist < distance){
              index = j ;
              distance = dist ;
          }
      }
      observations[i].id = index ;
    }


}

double Weight(vector<double> state_x , vector<double> state_y , double std_landmark[] ,vector<int> map_indices ,  const vector<LandmarkObs> landmarks){

  /*
  define the multivariant gaussian
  gauss_2d = (1/2*pi*stdev_x*stdev_y) * exp( - (x - mean_x )**2 /2*stdev_x**2  + ( y- mean_y)**2 / 2*stdev_y**2 )
  */
  double weight = 1.0;
  double coeff = 1/( 2* M_PI * std_landmark[0] * std_landmark[1]);
  double stdev_x2 = pow(std_landmark[0],2);
  double stdev_y2 = pow(std_landmark[1],2);
  if((state_x.size() == state_y.size())  ){
    for (int i =0 ; i<state_x.size() ; i++){
        double mean_x = state_x[i];
        double mean_y = state_y[i];
        //check on the id and get the matches based on id's
        int index = map_indices[i];
        //std::vector<int>::iterator itr = std::find(map_indices.begin() , map_indices.end() , index);
        //int map_index = std::distance(map_indices.begin(),itr);
        double x_map = landmarks[index].x ;
        double y_map = landmarks[index].y ;
        double value =pow(mean_x - x_map , 2.0)/( 2.0*stdev_x2 )  +  pow( mean_y - y_map ,2.0)/(2.0*stdev_y2);
        double val = coeff * exp(  -1.0 * (  pow(mean_x - x_map , 2.0)/( 2.0*stdev_x2 )  +  pow( mean_y - y_map ,2.0)/(2.0*stdev_y2) ) );
        weight *= val ;
      

    }

    return weight ;
  }
  else{
    std::cout << "Dimensiona are not match"<<std::endl;
  }


}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
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
    //for each particle we need to transform the vehicle observation as thier own observation
    //before add the observation into the map space incorparte with the gaussian measurement noise
    std::cout << "update weights "<<std::endl;
    for (u_int i=0 ; i < particles.size() ; i++){
      //obtain the each particle x , y , theta
      Particle particle ;
      particle = particles[i];
      double x = particle.x ;
      double y = particle.y ;
      double theta = particle.theta;
      //transform the each measurement
      /*
      [xm , ym , 1]= | cos(theta) -sin(theta)  x_p |  * [x_ob , y_ob ,1]
                     | sin(theta) cos(theta) y_p |
                     | 0           0          1  |

      */
      //create a new obseravtion vector
      vector<LandmarkObs> particle_obs ;
      for (int j =0 ; j<observations.size() ; j++){       
        double p_x = x +  ( cos(theta) * observations[j].x )  - (sin(theta) * observations[j].y);
        double p_y = y +  (cos(theta) * observations[j].y) + (sin(theta) * observations[j].x);
        LandmarkObs particle_landmark ;
        particle_landmark.x = p_x ;
        particle_landmark.y =p_y ;
        particle_obs.push_back(particle_landmark);
        //set the particle sense meassurements
        particle.sense_x.push_back(p_x);
        particle.sense_y.push_back(p_y);
      }
    

      //associte the particle sense with world landmark
      //find the world map landmarks within the sensor range
      vector<LandmarkObs> landmarkPred ;
      vector<int> map_indices;
      for (int k =0  ; k<map_landmarks.landmark_list.size();k++){
        double p_x = particle.x ; 
        double p_y = particle.y ;
        float land_x = map_landmarks.landmark_list[k].x_f;
        float land_y = map_landmarks.landmark_list[k].y_f ;
        int land_id = map_landmarks.landmark_list[k].id_i ;
        double dist = sqrt( pow(p_x - land_x , 2) + pow( p_y - land_y , 2) );
        if(dist < sensor_range){
          
            LandmarkObs landmarkobj ;
            landmarkobj.x = land_x ;
            landmarkobj.y = land_y ;
            landmarkobj.id = land_id;
            landmarkPred.push_back(landmarkobj);
        }
      }

      //call the data association function to map each particle measurement to map landmarks
      dataAssociation(landmarkPred , particle_obs);

      //map the id's into the particle state
      for(int l =0 ; l< particle_obs.size() ; l++){
        particle.associations.push_back(particle_obs[l].id);
        map_indices.push_back(particle_obs[l].id);
      }
      particles[i].weight = Weight(particle.sense_x  , particle.sense_y , std_landmark ,map_indices,landmarkPred);
      std::cout <<"Particle weight "<<particle.weight <<std::endl;
    }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    //define the resampling
    vector<double> particle_weights ;
    for (int i =0; i<particles.size();i++){
      particle_weights.push_back(particles[i].weight);
    }
    //random generator
    std::default_random_engine gen;
    //define the random sampler weighted on the particle weights  #################### NOTE #########################################
    std::discrete_distribution<> distribution(particle_weights.begin() , particle_weights.end());

    //define a new particle vector
    vector<Particle> resample_particles;
    for (int j =0 ;j < particles.size();j++){
      //redefine the particle vector
      Particle resamplePat = particles[distribution(gen)];
      resample_particles.push_back(resamplePat);
    }
    //assign the new particles to the particle vector
    for (int k =0 ; k < particles.size();k++){
      particles[k] = resample_particles[k];
    }


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
  particle.associations= associations;
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