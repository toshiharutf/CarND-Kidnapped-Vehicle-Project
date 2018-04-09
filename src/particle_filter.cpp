/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
#include "particle_filter.h"

using namespace std;

std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	num_particles = 100;
	//weights.resize(num_particles, 1.0);
	particles.reserve(num_particles);  // reserve mem space

	// This line creates a normal (Gaussian) distribution for x
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {
		
		double sample_x, sample_y, sample_theta;
		
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);

		Particle particle;
		particle.id = i;		
		particle.x = sample_x;
		particle.y = sample_y;
		particle.theta = sample_theta;
		particle.weight = 1.0;

		particles.push_back(particle);
                weights.push_back(1.0);
	}

    is_initialized = true;
    cout << "Initialization complete!" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	for (auto& p : particles){

		if (fabs(yaw_rate) > 0.001) {
			p.x      = p.x + velocity/yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
			p.y      = p.y + velocity/yaw_rate * (cos(p.theta)  - cos(p.theta + yaw_rate * delta_t));
			p.theta  = p.theta + yaw_rate * delta_t;
                        
                        // angle normalization
                        //while (p.theta> M_PI) p.theta-=2.*M_PI;
                        //while (p.theta<-M_PI) p.theta+=2.*M_PI;
                        
		} 
		else {
			p.x = p.x + velocity * delta_t * cos(p.theta);
			p.y = p.y + velocity * delta_t * sin(p.theta);
		}

		std::normal_distribution<double> dist_x(p.x, std_pos[0]);
		std::normal_distribution<double> dist_y(p.y, std_pos[1]);
		std::normal_distribution<double> dist_theta(p.theta, std_pos[2]);

		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
            }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>& predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

std::vector<LandmarkObs> ParticleFilter::associate_Observations(std::vector<LandmarkObs> inRangeLandmarks, std::vector<LandmarkObs> observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark.
    std::vector<LandmarkObs> associated_Observations;
    LandmarkObs closest;

    for (auto obs: observations){

        double shortest = 1E10; // initial large number 
        double distance = 0;
        for (auto l: inRangeLandmarks){
            distance = dist(obs.x,obs.y,l.x,l.y);
            if (distance < shortest) {
                shortest = distance;
                closest.x = obs.x;
                closest.y = obs.y;
                closest.id = l.id;  // the landmark id is associated to this observation
            }
        }
        associated_Observations.push_back(closest);
        //cout << "Closest landmark " << obs.id << "  distance: " << distance << endl;
    }

    return associated_Observations;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];

    for(int i=0; i < num_particles; ++i) {
        Particle p = particles[i];
        
        // Filter landmarks outside the particle's range
        std::vector<LandmarkObs> inRangeLandmarks;
        for(auto landmark: map_landmarks.landmark_list){
            if( dist(landmark.x_f, landmark.y_f, p.x, p.y) < sensor_range){
                LandmarkObs l;
                l.x = landmark.x_f;
                l.y = landmark.y_f;
                l.id = landmark.id_i; //why the struct is different?
                inRangeLandmarks.push_back(l);  // add the landmark inside the sensor's range
            }
        }

        // transform observations from particle's to map's coordinate system
        std::vector<LandmarkObs> transformed_Observations;
        for (auto obs: observations){

            LandmarkObs transformed_Obs;
            transformed_Obs.x  = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
            transformed_Obs.y  = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
            transformed_Obs.id = obs.id;
            transformed_Observations.push_back(transformed_Obs);
        }
        
        // Associate each measurement to a landmark in the map
        vector<LandmarkObs> associated_Observations;
        associated_Observations = associate_Observations(inRangeLandmarks, transformed_Observations);
                
        // assign a weight proportionally to its probability
        double prob = 1;
        for(auto obs: associated_Observations){
            double l_x = map_landmarks.landmark_list[obs.id-1].x_f;
            double l_y = map_landmarks.landmark_list[obs.id-1].y_f;
            
            prob *= mgp(obs.x, l_x, std_x, obs.y, l_y, std_y);
            //cout << "obs.id: " << obs.id << "  obs.x: " << obs.x  << "l_x: " << l_x << "  obs.y: " << obs.y  << "l_y: " << l_y <<endl;
        }
        p.weight = prob;
        weights[i] = prob;
        
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    
    std::vector<Particle> resampled_particles(num_particles); // empty vector for the resampled weights
    
    std::discrete_distribution<int> indexes(weights.begin(), weights.end());
    int index = indexes(gen);  // pick a random index to begin
    double beta = 0.0;
    double mw = *max_element(weights.begin(), weights.end());

    //std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0,1);
    
    for(int i = 0; i < num_particles; ++i){
        double r = distribution(gen);
        beta = r*2*mw;
        
        while(beta > weights[index]){
             beta -= weights[index];
             index = (index+1)%num_particles;
             //cout << "chosen index: " << index << endl;
        }
        resampled_particles[i] = particles[index];
    }
    
    particles = resampled_particles;
    

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

double ParticleFilter::mgp(double x,double mu_x,double  sigma_x,double  y,double  mu_y,double sigma_y){
    double constant = 1/(2*M_PI*sigma_x*sigma_y);

    return constant*exp(-( (x-mu_x)*(x-mu_x)/(2*sigma_x*sigma_x) + (y-mu_y)*(y-mu_y)/(2*sigma_y*sigma_y) ));
}
