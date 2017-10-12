#include <ros/ros.h>
#include <vector>
#include <array>
#include <cmath>
#include <detect/PanTilt.h>
#include <detect/ArenaPosition.h>
#include <detect/PanTilts.h>
#include <detect/ArenaPositions.h>
#include <gaml-libsvm.hpp>

typedef std::pair<double,double> PT;

class localize {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber pantilt_sub;
        ros::Publisher position_pub;
		
		std::string pantilts_topic;
		std::string arenapositions_topic;
		std::string path_to_model_x;
		std::string path_to_model_y;
		
		gaml::libsvm::Predictor<PT,double> predictor_x;
		gaml::libsvm::Predictor<PT,double> predictor_y;

    public:
		
		static int nb_nodes_of(const PT& pt) {
		  return 3;
		}

		static void fill_nodes(const PT& pt,struct svm_node* nodes) {
		  nodes[0].index = 1;
		  nodes[0].value = pt.first;  // pan
		  nodes[1].index = 2;
		  nodes[1].value = pt.second; // tilt
		  nodes[2].index = -1;        // end
		}
		
        localize() : nh_("~") {
			nh_.param("pantilts_topic",pantilts_topic,std::string("/detect/PanTilts"));
			nh_.param("arenapositions_topic",arenapositions_topic,std::string("/ArenaPositions"));
			nh_.param("path_to_model_x",path_to_model_x,std::string("x.pred"));
			nh_.param("path_to_model_y",path_to_model_y,std::string("y.pred"));
            pantilt_sub = nh_.subscribe(pantilts_topic,1,&localize::callback_pantilts,this);
            position_pub = nh_.advertise<detect::ArenaPositions>(arenapositions_topic,1);
            
            gaml::libsvm::Predictor<PT,double> pred(&localize::nb_nodes_of, &localize::fill_nodes);
            predictor_x = pred;
            predictor_y = pred;
            
            // Loading of the svm models previously trained.
            predictor_x.load_model(path_to_model_x);
            predictor_y.load_model(path_to_model_y);
        }
        
        void callback_pantilts(const detect::PanTilts& msg){
			std::vector<detect::PanTilt> pantilt_vect = msg.pantilts;
			detect::ArenaPositions positions;
			// For all pantilts, we use the model for x and the model for y to compute the corresponding arenaposition
			for(auto pantilt: pantilt_vect){
				detect::ArenaPosition position;
				std::pair<double,double> paire;
				paire.first = pantilt.pan;
				paire.second = pantilt.tilt;
				position.x = predictor_x(paire);
				position.y = predictor_y(paire);
				positions.arenapositions.push_back(position);
			}
			// Publication of the arenapositions message
			position_pub.publish(positions);
		}		
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"localize");
    localize lo;
	ros::spin();
    return 0;
}
