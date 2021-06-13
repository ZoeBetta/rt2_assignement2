/** @ package rt2_assignment2
* 
*  \file position_service.cpp
*  \brief This file implements the random number server
*
*  \author Zoe Betta
*  \version 1.0
*  \date 12/06/2021
*  \details
*   
*  Subscribes to: <BR>
*	 None
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    /position_server
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*    This node implements the server that returns a random position and 
* 	 orientation for the robot between given intervals. 
*/
#include "ros/ros.h"
#include "rt2_assignment2/RandomPosition.h"

/**
 * \brief: It generates a random number
 * \param M: the lower bound of the interval I want to have the random number in
 * \param N: the upper bound of the interval I want to have the random number in
 * 
 * \return: the random number generated
 * 
 * This function uses the library function rand() to generate a random number and
 *  then resizes it to be in the interval [M;N].
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * \brief: Called when I have a request from the client
 * \param req : it is a message composed of the maximum and minimum of the intervalin both x and y
 * \param res : it is a message composed of the random x and y coordinates and the orientation
 * 
 * \return: True
 * 
 * This function calls the function randMtoN to generate three random numbers,
 * the x coordinate, the y coordinate and the orientation that is said to be
 * requested between -pi and pi.
 */
bool myrandom (rt2_assignment2::RandomPosition::Request &req, rt2_assignment2::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief: The main function
 * 
 * \return: 0
 * 
 * This function initializes the ros node and the server /position_server,
 * it then runs waiting for a request for the server.
 */
int main(int argc, char **argv)
{
	// initialization of the node
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   // initialization of the server
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
