#include <osm_lidar.h>

namespace osm_localizer{


  class OsmLidarNode: OSMLidar{
  public:

      OsmLidarNode(ros::NodeHandle & n) : OSMLidar(n){

      }

      void update(){
        //  localization.updatePoseFromTF();
      }

  private:




  };
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "osm_localizer");
    ros::NodeHandle n;

    osm_localizer::OsmLidarNode osm_planner( n);
    ros::spin();



return 0;
}


