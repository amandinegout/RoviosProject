#include <gaml-libsvm.hpp>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <utility>
#include <string>
#include <fstream>
#include <iostream>
#include <ctime>

typedef std::pair<double,double> XY;
typedef std::pair<double,double> PT;
typedef std::pair<PT,XY>          Data;
typedef std::vector<Data>        DataSet;

class Loss {
public:
  typedef XY output_type;
  double operator()(const output_type& l1, const output_type& l2) const {
    gaml::loss::Quadratic<double> loss;
    return loss(l1.first, l2.first) + loss(l1.second, l2.second);
  }
};

int nb_nodes_of(const PT& pt) {
  return 3;
}

void fill_nodes(const PT& pt,struct svm_node* nodes) {
  nodes[0].index = 1;
  nodes[0].value = pt.first;  // x 
  nodes[1].index = 2;
  nodes[1].value = pt.second; // y
  nodes[2].index = -1;        // end
}

const PT& input_of (const Data& data) {return data.first;} // this should return a double for libsvm.
const XY& output_of(const Data& data) {return data.second;} // this should return a double for libsvm.
double x_of(const Data& data) {return data.second.first;} // this should return a double for libsvm.
double y_of(const Data& data) {return data.second.second;} // this should return a double for libsvm.


// Gnuplot function to generate the plot files for x and y models according to pan tilt values
#define PLOT_STEP 1
#define PAN_MIN 10
#define PAN_MAX 90
#define TILT_MIN -70
#define TILT_MAX -10

template<typename Func>
void gnuplot(std::string filename_x, std::string title_x ,std::string filename_y, std::string title_y ,const Func& f) {
	
  PT pt;
  std::ofstream file_x;
  std::ofstream file_y;
  file_x.open(filename_x);
  file_y.open(filename_y);
  
  if(!file_x) {
    std::cerr << "Cannot open \"" << filename_x << "\"." << std::endl;
    return;
  }
  if(!file_y) {
    std::cerr << "Cannot open \"" << filename_y << "\"." << std::endl;
    return;
  }
  file_x << "set hidden3d" << std::endl
       << "set title \"" << title_x << "\"" << std::endl
       << "set view 41,45" << std::endl
       << "set xlabel \"pan\"" << std::endl
       << "set ylabel \"tilt\"" << std::endl
       << "set zlabel \"x\"" << std::endl
       << "set ticslevel 0" << std::endl
       << "splot '-' using 1:2:3 with lines notitle" << std::endl;
       
  file_y << "set hidden3d" << std::endl
       << "set title \"" << title_y << "\"" << std::endl
       << "set view 41,45" << std::endl
       << "set xlabel \"pan\"" << std::endl
       << "set ylabel \"tilt\"" << std::endl
       << "set zlabel \"y\"" << std::endl
       << "set ticslevel 0" << std::endl
       << "splot '-' using 1:2:3 with lines notitle" << std::endl;
  
  for(pt.first=PAN_MIN;pt.first<=PAN_MAX;pt.first+=PLOT_STEP,file_x << std::endl)
    for(pt.second=TILT_MIN;pt.second<=TILT_MAX;pt.second+=PLOT_STEP,file_x << std::endl)
      file_x << pt.first << ' ' << pt.second << ' ' << f(pt).first;
  file_x.close();
  
  for(pt.first=PAN_MIN;pt.first<=PAN_MAX;pt.first+=PLOT_STEP,file_y << std::endl)
    for(pt.second=TILT_MIN;pt.second<=TILT_MAX;pt.second+=PLOT_STEP,file_y << std::endl)
      file_y << pt.first << ' ' << pt.second << ' ' << f(pt).second;
  file_y.close();
  
  std::cout << "Gnuplot file \"" << filename_x << "\" generated." << std::endl;
  std::cout << "Gnuplot file \"" << filename_y << "\" generated." << std::endl;
}

// Computation of the number of samples given in position.txt
int number_samples(const char* filename){
	std::ifstream file;
	file.open(filename,std::ios::in);
	if(!file) {
		std::cerr << "Cannot open \"" << filename << "\"." << std::endl;
		return 1;
	}
	int lines_count =0;
	std::string line;
	while (std::getline(file , line))
		++lines_count;
	return lines_count;
}

std::array<double,2> array_of_output(const XY& output) {
  return {{output.first,output.second}};
}

XY output_of_array(const std::array<double,2>& values) {
  XY res;
  res.first = values[0];
  res.second = values[1];
  return res;
}

int main(int argc, char* argv[]) {
	
  // Let us make libsvm quiet
  gaml::libsvm::quiet();
  
  if(argc != 2) {
    std::cerr << "Usage : " << argv[0] << " for data-samples file" << std::endl;
    return 1;
  }
  
  int nb_samples = number_samples(argv[1]);
  std::cout << "There are :" << nb_samples << " samples" << std::endl;
  
  try {
    // Let us collect samples.
    
    DataSet basis;
    DataSet::iterator iter,end;
	// Resizing the basis according to the number of samples
    basis.resize(nb_samples);
    std::ifstream dataset;
    dataset.open(argv[1],std::ios::in);
	if(!dataset) {
		std::cerr << "Cannot open \"" << argv[1] << "\"." << std::endl;
		return 1;
	}
	// Filling the basis with the dataset
	for(iter = basis.begin(), end = basis.end(); iter != end; ++iter) {
      double x,y,p,t;
	  dataset >> p >> t >> x >> y;
      XY xy(x,y);
      PT pt(p,t);
      *iter = Data(pt,xy);
    }
    dataset.close();
    
    // Let us set configure a svm
    struct svm_parameter params;
    gaml::libsvm::init(params);
    params.kernel_type = RBF;          // RBF kernel
    params.gamma       = 0.002;           // k(u,v) = exp(-gamma*(u-v)^2)
    params.svm_type    = EPSILON_SVR;
    params.p           = 0.1;          // epsilon
    params.C           = 120;
    params.eps         = 0.04;         // numerical tolerence
    
	// This sets up a svm learning algorithm for predicting a scalar.
    auto scalar_learner = gaml::libsvm::supervized::learner<PT,double>(params, nb_nodes_of, fill_nodes);

    // Let us use it for learning x and y.
    auto learner = gaml::multidim::learner<XY,2>(scalar_learner, array_of_output, output_of_array);

    // Let us train it and get some predictor f. f is a function, as the oracle.
    std::cout << "Learning..." << std::endl;
    auto f = learner(basis.begin(),basis.end(),input_of,output_of);
	
	// Let us retrieve the three SVMs in order to save each one.
    auto f_predictors = f.predictors();
    std::array<std::string,2> filenames = {{std::string("x.pred"),"y.pred"}};
    auto name_iter = filenames.begin();
    for(auto& pred : f_predictors) pred.save_model(*(name_iter++));
	
    // Let us plot the result.
    gnuplot("pantiltmodel_x.plot","SVM model of the pan tilt for x","pantiltmodel_y.plot","SVM model of the pan tilt for y",f);

    // All libsvm functions related to svm models are implemented.
    auto pred_iter = f_predictors.begin();
    auto& x_pred = *(pred_iter++);
    auto& y_pred = *(pred_iter++);
    for(auto& pred : f_predictors){
		std::cout << std::endl
				  << "There are " << pred.get_nr_sv() << " support vectors." << std::endl;
    }
    
     // We can compute the empirical risk with gaml tools.
    auto evaluator = gaml::risk::empirical(gaml::loss::Quadratic<double>());
    double risk = evaluator(x_pred, basis.begin(), basis.end(), input_of, x_of);
    std::cout << "Empirical risk : " << risk << std::endl
	      << "    sqrt(risk) : " << sqrt(risk) << std::endl;
    
    risk = evaluator(y_pred, basis.begin(), basis.end(), input_of, y_of);
    std::cout << "Empirical risk : " << risk << std::endl
	      << "    sqrt(risk) : " << sqrt(risk) << std::endl;
    
    bool verbosity = true;
    auto leave_one_out_evaluator = gaml::risk::cross_validation(Loss(), gaml::partition::leave_one_out(), verbosity);
    double l1o_risk = leave_one_out_evaluator(learner, basis.begin(), basis.end(), input_of, output_of);
    std::cout << std::endl 
	    << "Estimation of the real risk (leave one out): "  << l1o_risk   << std::endl;
    
    std::cout << "test" << std::endl ;
    for (auto& data : basis){
		std::cout << x_of(data) << " " << x_pred(input_of(data)) << " " << y_of(data) << " " << y_pred(input_of(data)) << " " << Loss()(output_of(data), f(input_of(data))) << std::endl ;
	}
   
	
    
  }
  catch(gaml::exception::Any& e) {
    std::cout << e.what() << std::endl;
  }
  
  return 0;
}
