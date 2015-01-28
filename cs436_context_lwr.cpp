#include <cs436_lwr/cs436_context.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <map>
#include <math.h>

//  Hau Sze Homework 3
//  April 17th 2014

CS436Context::CS436Context( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

  CS436Context::~CS436Context(){}

bool CS436Context::solve( planning_interface::MotionPlanResponse &res )
{
  
  std::cout << std::setprecision(2) << std::fixed;  // set the fixed decimal points 
  srand (time(NULL));  // initialize random seed
  
  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, 
							      getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints to the vectors qfin and qini (convenience)
  std::vector<double> qfin;
  std::vector<double> qini;

  for( size_t i=0; i<robotmodel->getVariableCount(); i++ )
  {
    qfin.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qini.push_back(request_.start_state.joint_state.position[i]);
  }

std::cout << "The goal is " << qfin[0] << " , " << qfin[1] << " , " << qfin[2] 
<< " , " << qfin[3] << " , " << qfin[4] << " , " << qfin[5] << " , " << qfin[6] << std::endl;


  // get access to the world. the world contains the obstacles
  collision_detection::WorldConstPtr world = getPlanningScene()->getWorld();
  
  // if the world not empty (at least one object)
  if( 0 < world->size() )
  {

    // Iterate over all the objects (if any) and print stuff each object
    collision_detection::World::const_iterator it;
    for( it=world->begin(); it!=world->end(); it++ )
    {
      // get the pointer to the object data
      collision_detection::World::ObjectConstPtr object = it->second;
      // print the ID number of the object
      std::cout << "Object ID: " << object->id_ << std::endl;

      // print the position/orientation of all the shapes that constitute the
      // object (data type is EigenSTL::vector_Affine3d)
      for( size_t i=0; i<object->shape_poses_.size(); i++ )
      {
	Eigen::Affine3d::MatrixType matrix = object->shape_poses_[i].matrix();
	// the position (translation) is at elements
	std::cout << "Position of object: [ " 
		  << std::setw(10) << matrix(0,3) 
		  << std::setw(10) << matrix(1,3) 
		  << std::setw(10) << matrix(2,3) 
		  << " ]" << std::endl;
      }
    }

  }
  else
  { std::cout << "Empty world" << std::endl; 
  }

  //  THE RRT LOOP,  RRT trajectory generation in configuration space
  bool notgoal = true;  // a bool use by the loop to determine when to stop
  int counter = 0;  // a node counter to prevent overflow

  //  Define the tree w/ parent value and initialize the first node w/ the initial q
  std::map< std::vector<double>, std::vector<double> > tree;
  tree[qini] = qini;
  //  Define a keeper to keep track of the level and number of nodes in each level in the tree
  std::map< std::pair<int, int>, std::vector<double> > keeper;
  keeper[std::make_pair(0,0)] = qini;
  int np = 1;  // dummy node counter for the last level
  int nl = 1;  // dummy level counter
  int nk = 0;  // dummy node counter for the current level
  //  Define the end node (use to retrieve the trajectory) and the previous(parent) node, nodep.
  std::vector<double> endnode;
  std::vector<double> nodep = qini;

  // define bestdist, the best distance to goal so far, to be a criteria to accept new nodes
  double bestdist = pow((nodep[0]-qfin[0]),2) + pow((nodep[1]-qfin[1]),2) + pow((nodep[2]-qfin[2]),2) + pow((nodep[6]-qfin[6]),2);
  bestdist = bestdist + pow((nodep[3] - qfin[3]),2) + pow((nodep[4] - qfin[4]),2) + pow((nodep[5] - qfin[5]),2);

  //  A while loop on the tree until a node reaches the goal or counter/level max out
  while ( notgoal && counter < 10000 && nl < 30)
  {
    //  define the tol, tolerance as a quadratic function, to modify the criteria on accepting new node
    double tol = (-116*nl*nl + 3601*nl - 620 )/2900; 
    //std::cout << "Current level is " << nl << " tol is " << tol << " node count is " << counter << std::endl; // debug

    //  Increase the level count
    nl++;

    //  Loop for new node on the nodes on the lowest level so far
    for (int i=0; i<np; i++)
    {
      	//  Prepare the parent vector;  
	
      	nodep = keeper[std::make_pair((nl-2),i)];

    	  for (int di=0; di<3; di++)  // iterating for some new nodes on each parent node
    	  {
    	    //  Generate a new vector or few vectors
    	    std::vector<double> qi; 
    	    //TO-DO how to generate few vectors
    	    for ( int k=0; k<qfin.size(); k++)
    	    {
		double iran = rand() % 100;  // random variable generator, 1 to 99
		double iran2 = (iran-50)/100;  // put iran as a ratio with random sign 
		double dnl = 10/nl + 1;
		double dp;
		if (nl < 5) //  The key of RRT is the method to sample.  If no solution, can modify the constant here
		{
			dp = (qfin[k] - nodep[k])*(1/nl)  + 3.14/2*iran2/nl;
		}
		else
		{
			dp = (qfin[k] - nodep[k])*(1 + 3.14/2*iran2);
		}
		qi.push_back(nodep[k] + dp);
    	    }
    //std::cout << "<< CHECK POINT!"  << std::endl;  //debug

    	    // Test the new vector for collision (also the collision along the path to it)
    	    bool collision = false;
    	    for (double t=0.0;  t<=1.0;  t+=0.33)  //  check every 1/3 distance to the new node for collision
    	    {
      	  	std::vector<double> qii;
      	  	for ( int j=0; j<qfin.size(); j++ )
    	  	{  
	    	  // interpolate between parent node and new node for collision check
            	  qii.push_back( ( 1.0 - t)*nodep[j] + t*qi[j] ); 
    	  	}

		//  Using the given collision detection program to check for collision
    	  	// create a robot state
     	  	moveit::core::RobotState robotstate( robotmodel );
    	  	// set the joint positions
    	  	robotstate.setJointGroupPositions( "robot", qii );
    	  	// if state is colliding, break and reject this node
    	  	if( getPlanningScene()->isStateColliding( robotstate ) )
    	  	{ 
	      	  collision = true;
	      	  break;
    	  	}
	     }
	     //  Test for whether the new node getting closer to the goal
    	     double gdistance = pow((qi[0] - qfin[0]),2) + pow((qi[1] - qfin[1]),2) + pow((qi[2] - qfin[2]),2) + pow((qi[3] - qfin[3]),2) + pow((qi[4] - qfin[4]),2) + pow((qi[5] - qfin[5]),2)+ pow((qi[6] - qfin[6]),2);

	//	std::cout << "<< check point " << gdistance << std::endl;  //debug

      	     // If collision free, add the new node into the tree and update the keeper
      	     //if (collision == false && gdistance < (bestdist * tol) ) 
	     if (collision == false  ) 
      	     {  
	  	counter++;

	   	tree[qi] = nodep; 
	  	keeper[std::make_pair(nl-1,nk)] = qi;
	  	nk++;
  
	  	if (bestdist > gdistance)
	  	{
	    	  bestdist = gdistance;
	  	}

	  	// If the distance is close enough to the goal, break out
    	  	if (gdistance < 0.1) 
    	  	{  
	     	  notgoal = false;
	     	  endnode = qi;
    	  	}
      	     }
	     // break out if a node very near the goal is found
	     if (notgoal == false) {break;}
      	}
     	     

	//  The end of a parent node iteration for n
	if (notgoal == false) {break;}
    }
    	//  as we going into the next level, update np (the previous # of nodes in previous level)
    	np = nk;
    	//  reset the counter for the # of nodes in the next level
    	nk = 0;
//std::cout << "the last generation is " << np << " bestdist is " << bestdist << std::endl;  //debug
  }



  //  Extract the trajectory from the tree
  //  Use the number of level as a divider to get an about fixed amount of steps in the trajectory
  int nstep = 30/(nl);
  //  Retrive all the via points from the tree into another map in reverse order
  std::map< int, std::vector<double> > traj;
  traj[nl] = qfin;
  traj[nl-1] = endnode;

std::cout << "lowest level is " << nl << " and nstep is " << nstep << std::endl;  //debug


  //  Reverse the trajectory for convenience
  for (int ii=0; ii<(nl-1) ; ii++) 
  {    
	int qq = nl-2-ii;
	std::vector<double> ttt = tree[ traj[(qq+1)] ];
	std::cout << "ii is " << ii << " qq is " << qq << " and " << ttt[0] << std::endl; 
    	traj[qq] = ttt;
  }

  //  Load the trajectory, start to goal
  for (int i=0; i<nl; i++)
  {
    for (double t=0.0;  t<=1.0;  t+=(1.0/nstep))
    {
      std::vector<double> qiii;
      for ( int j=0; j<qfin.size(); j++ )
      {  
	// interpolate between parent via node and via node
        qiii.push_back( ( 1.0 - t)*traj[i][j] + t*traj[i+1][j] ); 
      }

 std::cout << "This step is " << qiii[0] << " , " << qiii[1] << " , " << qiii[2] << " , " << qiii[3] << " , " << qiii[4] << " , " << qiii[5] << std::endl;  

     // create a robot state
     moveit::core::RobotState robotstate( robotmodel );
     // set the joint positions
     robotstate.setJointGroupPositions( "robot", qiii );

     for( size_t j=1; j<=robotmodel->getVariableCount(); j++ )
     {
     	// Add the robot state as a way point
     	res.trajectory_->addSuffixWayPoint( robotstate, 0.0 );
     }
  }
}

  res.error_code_.val = 1;

  return true;

}

bool CS436Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS436Context::clear(){}

bool CS436Context::terminate(){return true;}
