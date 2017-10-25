/**
 *  This source file implements the main function that runs the ALLIANCE
 * LowLevelNode class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/low_level_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "low_level_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  nodes::LowLevelNodePtr node(new nodes::LowLevelNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
