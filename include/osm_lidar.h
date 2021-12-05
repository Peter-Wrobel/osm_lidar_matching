/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu), Michal
 * WEBSITE: https://www.BrucebotStudio.com/
 */

#pragma once 

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_space.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>

#include <deque>
#include <stack>
#include <tuple>
#include <cmath>
//reconfigure
#include <dynamic_reconfigure/server.h>
#include <osm_localization/osm_localizationConfig.h>


//ros and tf
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

//messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>


typedef double dist_t;
typedef double period_t;






namespace osm_localizer{

    class OSMLidar{
    public:
        OSMLidar( ros::NodeHandle & n);

        /*main functions*/
        void getVelodyneCB(const sensor_msgs::PointCloud2ConstPtr & msg);
        bool getGroundPlane(void);
        void makeEdges(const ros::TimerEvent&);
        void filterNonGround(const ros::TimerEvent&);
        void makeRings(void);
        void analyzeRings(void);


        /*helpers*/
        static pcl::PointXYZ getCentroid(const std::vector<int> & pointIdxRadiusSearch,
                                         const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud);

        /*dynamic reconfigure*/
        void reconfigCB(osm_localization::osm_localizationConfig &config, uint32_t level);



    private:

        ros::Subscriber pc_sub_;
        std::string     pc_topic_;

        ros::Publisher  filtered_pc_pub_;
        ros::Publisher  ring_pub_;


        dynamic_reconfigure::Server<osm_localization::osm_localizationConfig> server_;
        static uint     DECK_SIZE;
        static float    NORM_THRESHOLD; //== cosine(theta)
        static period_t PERIOD;
        static dist_t   CENTROID_DIST;
        static dist_t   SEARCH_RADIUS;
        static int     RING_ID;

        ros::Timer      edge_timer_;
        ros::Timer      road_timer_;
        std::deque <pcl::PointCloud<pcl::PointXYZ>::Ptr>  cloud_deck_;

        std::map <int, std::vector<pcl::PointXYZ>> ring_map_;
        std::map <int, pcl::PointCloud<pcl::PointXYZ>::Ptr> ring_fanc_map_;


        pcl::PointCloud<pcl::PointXYZ>::Ptr  filtered_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr  edge_cloud_;
        

    };

}