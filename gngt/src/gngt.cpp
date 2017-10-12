#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vq2.h>
#include <utility>
#include <vector>
#include <map>
#include <ctime>
#include <cstdlib>
#include <iterator>
#include <detect/Entity.h>
#include <detect/Entities.h>

// ###############
// #             #
// # GNG-T Stuff #
// #             #
// ###############
//       ###
//       ###
//     #######
//      ##### 
//       ###
//        #

// This is inspired from vq2 examples.

// This defines the graph
typedef vq2::algo::gngt::Unit<pcl::PointXYZ>         Unit;
typedef vq2::Graph<Unit,char,Unit::copy_constructor> GNGT;

// For finding the winner in the GNGT algorithm, we need some
// similarity measure. Here, let us use the squared euclidian
// distance.
class Similarity {
public:
  typedef pcl::PointXYZ value_type;
  typedef pcl::PointXYZ sample_type;
  double operator()(const value_type& arg1,
		    const sample_type& arg2) {
    double dx = arg1.x - arg2.x;
    double dy = arg1.y - arg2.y;
    return dx*dx + dy*dy;
  }
};
typedef vq2::unit::Similarity<Unit,Similarity> UnitSimilarity;

// The learning process.
class Learn {
public:
  typedef pcl::PointXYZ sample_type;
  typedef pcl::PointXYZ weight_type;
  void operator()(double coef,
                  weight_type& prototype,
                  const sample_type& target) {
    prototype.x += coef * (target.x - prototype.x);
    prototype.y += coef * (target.y - prototype.y);
  }
};
typedef vq2::unit::Learn<Unit,Learn> UnitLearn;

// This is the parameter set for GNG-T
struct Params {
public:
  // GNG-T
  int ageMax(void)           {return 15;}
  double learningRate(void)  {return .005;}
  double learningRatio(void) {return .2;}
  double lambda(void)        {return .001;}
  // Evolution
  double target(void)        {return 1e-5;}
  double nbSamples(void)     {return 10000;}
  double lowPassCoef(void)   {return .4;}
  double delta(void)         {return 0.75;}
  double margin(void)        {return .2;}
};

// Some evolution class is required to control the GNG-T
// evolution. Let us use the default one here.
typedef vq2::by_default::gngt::Evolution<Params> Evolution;

// This functor collects vertices and edges.
class Get {
public:
  std::vector<pcl::PointXYZ>                             vertices;
  std::vector< std::pair<pcl::PointXYZ,pcl::PointXYZ> >  edges;
  
  bool operator()(GNGT::edge_type& e) { 
    if(e.stuff.efficient)
      edges.push_back(std::make_pair((*(e.n1)).value.prototype(),
				     (*(e.n2)).value.prototype()));
    return false;
  }

  bool operator()(GNGT::vertex_type& v) { 
    if(v.stuff.efficient)
      vertices.push_back(v.value.prototype());
    return false;
  }
};


//        #
//       ###
//      ##### 
//     #######
//       ###
//       ###
// ###############
// #             #
// # GNG-T Stuff #
// #             #
// ###############

// This method returns the coordinates of the center of the current convexe shape representing the robot
pcl::PointXYZ getCenter(std::vector<pcl::PointXYZ>& vertices_vect) {
	double mean_x=0.0;
	double mean_y=0.0;
	for (auto vert : vertices_vect){
		mean_x += vert.x;
		mean_y += vert.y; 
	} 
	mean_x /= vertices_vect.size();
	mean_y /= vertices_vect.size();
	pcl::PointXYZ center;
	center.x = mean_x;
	center.y = mean_y;
	center.z = 0.0;
	return center;
}

double uniform(double min, double max) {
  return min + (max-min)*(std::rand()/(1.0+RAND_MAX));
}

// This is a class for allocating a color to each graph component in a
// smart way.
struct Color {
  double r,g,b;
};

class Colormap {
private:
  std::map<unsigned int, Color> color_of;
  std::map<unsigned int, Color> tmp;

  Color new_color() {
    double a = uniform(0,1);
    double b = uniform(0,1);
    int idx = (int)(uniform(0,3));
    if(idx == 0) return {1.0,   a,   b};
    if(idx == 1) return {  a, 1.0,   b};
    else         return {  a,   b, 1.0};
  }

public:
  
  void remap() {
    tmp = color_of;
    color_of.clear();
  }
  
  void allocate(unsigned int label) {
    auto existing = tmp.find(label);
    if(existing != tmp.end())
      color_of[label] = existing->second;
    else
      color_of[label] = new_color();
  }
  
  Color operator()(unsigned int label) {
    return color_of[label];
  }
};

// This class helps to register marker idfs and clean unused ones.
class MarkerIdfs {
private:

  std::map<int,bool> idf_in_use;

public:

  void reset() {
    for(auto& key_val : idf_in_use) key_val.second = false;
  }

  void use(int idf) {
    idf_in_use[idf] = true;
  }

  void cleanup(ros::Publisher& pub) {
    std::vector<int> unused;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "gngt";
    marker.action = visualization_msgs::Marker::DELETE;
    
    for(auto& key_val : idf_in_use)
      if(!(key_val.second)) {
	unused.push_back(key_val.first);
	marker.id = key_val.first;
	pub.publish(marker);
      }
    
    for(auto idf : unused)idf_in_use.erase(idf);
  }

};

void draw_cylinder(pcl::PointXYZ& center, ros::Publisher& pub, const Color color, int idc);
void draw_vertices(ros::Publisher& pub,const Color color, int id, const std::vector<pcl::PointXYZ>& vertices);
void draw_edges(ros::Publisher& pub, const Color color, int id, const std::vector< std::pair<pcl::PointXYZ,pcl::PointXYZ> >& edges);

#define NB_STABILIZATION 5

// Tableau des ids utilisés pour les cylindres
std::vector<unsigned int> id_used;

void gngt_epoch_cb(std::vector<ros::Publisher>& pubs,
		   GNGT&                                   gngt,
		   Params&                                 params,
		   UnitSimilarity&                         distance,
		   UnitLearn&                              learn,
		   Evolution&                              evolution,
		   Colormap&                               cmap,
		   MarkerIdfs&                             marker_idfs,
		   const sensor_msgs::PointCloud2ConstPtr& input) {
  // Publisher of edges and vertices
  ros::Publisher pub = pubs[0];
  // Publisher of cylenders centered on the robot
  ros::Publisher pub_cyl = pubs[1];
  // Publisher of entities messages with label and arenaposition of the robot
  ros::Publisher pub_ent = pubs[2];
  
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::PCLPointCloud2            points_2;

  // Get the samples
  pcl_conversions::toPCL (*input,   points_2);
  pcl::fromPCLPointCloud2(points_2, points  ); 

  // Let us use few epochs to stabilize the current graph to the new
  // input (no graph evolution, only adjustment).
  for(int i=0; i< NB_STABILIZATION; ++i) {
    std::random_shuffle(points.begin(), points.end());
    vq2::algo::gngt::open_epoch(gngt,evolution);
    for(auto& pt : points) 
      vq2::algo::gngt::submit(params,gngt,
			      distance,learn,
			      pt,false);
    vq2::algo::gngt::close_epoch(params,gngt,
				 learn,
				 evolution,false);
  }

  // Update the graph (add or remove egdes and vertices)
  vq2::algo::gngt::open_epoch(gngt,evolution);
  for(auto& pt : points) 
    vq2::algo::gngt::submit(params,gngt,
			    distance,learn,
			    pt,true);
  vq2::algo::gngt::close_epoch(params,gngt,
			       learn,
			       evolution,true);

  
  // Update the connected components and their labels.
  std::map<unsigned int,GNGT::Component*> components;
  gngt.computeConnectedComponents(components,true);
  cmap.remap();
  for(auto& key_val : components) cmap.allocate(key_val.first);

  // Let us now send the graph to Rviz, component by component.
  marker_idfs.reset();
  detect::Entities entities_total;
  
  // Cleanning of the cylinder marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "cylinders";
  marker.action = visualization_msgs::Marker::DELETE;
  
  // Storage of new ids
  std::vector<unsigned int> new_ids;
  for(auto& key_val : components) {
	  new_ids.push_back(key_val.first);
	  ROS_INFO("Id : %d",(int)key_val.first);
  } 
  // Publication of each cylinder iff its id is found among new ids
  for(auto& id_courant : id_used){
	  if(std::find(new_ids.begin(), new_ids.end(), id_courant) == new_ids.end()) {
		  marker.id = id_courant;
		  pub_cyl.publish(marker);
	  }
  }
  // Update of currently used ids
  id_used = new_ids;
  
  for(auto& key_val : components) {
    Get get;
    int idf;
    auto label    = key_val.first;
    auto comp_ptr = key_val.second;
    comp_ptr->for_each_vertex(get);
    comp_ptr->for_each_edge(get);
    
    auto color = cmap(label);
    idf = 2*label; draw_vertices(pub,color,idf,get.vertices); marker_idfs.use(idf);
    ++idf;         draw_edges   (pub,color,idf,get.edges);    marker_idfs.use(idf);
    // Computation of the center of the convexe shape
    pcl::PointXYZ center = getCenter(get.vertices);
    // Publication of the corresponding cylinder
    draw_cylinder(center, pub_cyl, color, label);
    // Creation of an Entity message for the current robot/cylinder
    detect::Entity entity_single;
    entity_single.label = label;
    entity_single.arenaposition.x = center.x;
    entity_single.arenaposition.y = center.y;
    // Adding the entity to the entites array
    entities_total.entities.push_back(entity_single);
  }  
  marker_idfs.cleanup(pub);
  pub_ent.publish(entities_total);
  
}


int main(int argc, char **argv)
{
  GNGT             g;
  Params           params;
  Similarity       distance;
  UnitSimilarity   unit_distance(distance);
  Learn            learn;
  UnitLearn        unit_learn(learn);
  Evolution        evolution(params);
  Colormap         cmap;
  MarkerIdfs       marker_idfs;

  std::srand(std::time(0));

  ros::init(argc, argv, "gngt");
  ros::NodeHandle n;

  GNGT gngt;

  ros::Publisher  pub = n.advertise<visualization_msgs::Marker>("markers", 0);
  ros::Publisher  pub_cyl = n.advertise<visualization_msgs::Marker>("cylinders", 0);
  ros::Publisher  pub_ent = n.advertise<detect::Entities>("entities", 0);
  
  // Creation of a vector of publishers to handle the maximum number of parameters of boost:bind
  std::vector<ros::Publisher> pubs;
  pubs.push_back(pub);
  pubs.push_back(pub_cyl);
  pubs.push_back(pub_ent);
  
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("input", 1, 
							      boost::bind(gngt_epoch_cb,
									  boost::ref(pubs),
									  boost::ref(gngt),
									  boost::ref(params),
									  boost::ref(unit_distance),
									  boost::ref(unit_learn),
									  boost::ref(evolution),
									  boost::ref(cmap),
									  boost::ref(marker_idfs),
									  _1));
  ros::spin();

  return 0;
}

// RVIZ drawing stuff.

// Publication method of a cylinder
void draw_cylinder(pcl::PointXYZ& center, ros::Publisher& pub_cyl, const Color color, int idc){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "cylinders";
  marker.id = idc;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center.x;
  marker.pose.position.y = center.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;
  marker.pose.orientation.w = 0.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.5;
  marker.color.a = 1.0; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  
  pub_cyl.publish(marker);
}

void draw_vertices(ros::Publisher& pub, const Color color, int id, const std::vector<pcl::PointXYZ>& vertices) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "gngt";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  
  auto out = std::back_inserter(marker.points);
  for(auto& pt : vertices) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    *(out++) = p;
  }
  pub.publish(marker);
}

#define EDGE_DARKEN_COEF .5
void draw_edges(ros::Publisher& pub, const Color color, int id, const std::vector< std::pair<pcl::PointXYZ,pcl::PointXYZ> >& edges) {
  visualization_msgs::Marker marker;


  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "gngt";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.a = 1.0; 
  marker.color.r = color.r*EDGE_DARKEN_COEF;
  marker.color.g = color.g*EDGE_DARKEN_COEF;
  marker.color.b = color.b*EDGE_DARKEN_COEF;
  
  auto out = std::back_inserter(marker.points);
  for(auto& e : edges) {
    geometry_msgs::Point p;
    p.x = e.first.x;
    p.y = e.first.y;
    *(out++) = p;
    p.x = e.second.x;
    p.y = e.second.y;
    *(out++) = p;
  }
  pub.publish(marker);
}
