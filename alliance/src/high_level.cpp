/**
 *  This source file implements the main function that runs the ALLIANCE
 * HighLevelNode class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/high_level_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "high_level_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  nodes::HighLevelNodePtr node(new nodes::HighLevelNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
