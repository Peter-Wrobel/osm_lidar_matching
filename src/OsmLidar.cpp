#include <osm_lidar.h>

namespace osm_localizer{

        uint     OSMLidar::DECK_SIZE       = 7;
        float    OSMLidar::NORM_THRESHOLD  = 0.95; //== cosine(theta)
        period_t OSMLidar::PERIOD          = 4;
        dist_t   OSMLidar::CENTROID_DIST   = 1;
        dist_t   OSMLidar::SEARCH_RADIUS   = 0.2;


    OSMLidar::OSMLidar(ros::NodeHandle & n):
    filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>), 
    edge_cloud_    (new pcl::PointCloud<pcl::PointXYZ>) {

        if(!n.getParam("/osm_localizer/velodyne_topic", pc_topic_)){

            ROS_ERROR("[OSMLidar]: NO velodyne parameter");
            exit(1);
        }
        // 0. subscribers, timers publishers
        pc_sub_          = n.subscribe(pc_topic_, 5, &OSMLidar::getVelodyneCB, this);
        edge_timer_      = n.createTimer(ros::Duration(PERIOD), &OSMLidar::makeEdges, this);

        filtered_pc_pub_ = n.advertise<sensor_msgs::PointCloud2>("chatter", 3);

        // 1. dynamic reconfigure
        server_.setCallback(boost::bind(&OSMLidar::reconfigCB, this, _1, _2));

    }


    void OSMLidar::reconfigCB(osm_localization::osm_localizationConfig &config, uint32_t level){

        ROS_INFO_THROTTLE(1.0, "[OSM localization] new parameters requested");
        DECK_SIZE       = config.deck_size;
        NORM_THRESHOLD  = config.norm_threshold; 
        PERIOD          = config.period;
        CENTROID_DIST   = config.centroid_dist;        
    }


    bool OSMLidar::getGroundPlane(void){

        //0. No run if point cloud isn't here
        if(cloud_deck_.size()== 0){

            ROS_WARN_THROTTLE(1.0, "[OSM localization]: no point clouds yet");
            return false;
        }



        //1. Combine message clouds to one and pass to normal estimator
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::PointXYZ>::Ptr comb_pc(new pcl::PointCloud<pcl::PointXYZ>); 


        for(auto pc_deck_itr = cloud_deck_.begin(); pc_deck_itr!=cloud_deck_.end(); ++pc_deck_itr){
            *comb_pc += *(*pc_deck_itr);
        }



        ne.setInputCloud(comb_pc);


        // 2. Framework for searching Kd closest neighbors
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // 3. Get normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
        ne.setRadiusSearch (0.2);
        ne.compute (*cloud_normals);




        // 4. add points thawt meet normal && height threshold
        auto comb_pcl_ptr = comb_pc->begin();
        for( const pcl::Normal & norm : (*cloud_normals)){

            if(norm.normal_z > NORM_THRESHOLD && comb_pcl_ptr->z  < 1.0){

                filtered_cloud_->push_back(*comb_pcl_ptr);
            }

            ++comb_pcl_ptr;
        }
        filtered_cloud_->is_dense = true;

        return true;

    }



    pcl::PointXYZ OSMLidar::getCentroid(const std::vector<int> & pointIdxRadiusSearch,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud){

        float x_acc = 0;
        float y_acc = 0;
        float z_acc = 0;

        int N = (int) pointIdxRadiusSearch.size();

        // 0. accumulate points
        for(int pc_ind : pointIdxRadiusSearch){
            const pcl::PointXYZ  & p_now =  (*filtered_cloud)[pc_ind];

            x_acc +=p_now.x;
            y_acc +=p_now.y;
            z_acc +=p_now.z;
        }

        //1. return average

        return pcl::PointXYZ(x_acc/N, y_acc/N, z_acc/N);

    }


    void OSMLidar::makeEdges (const ros::TimerEvent&){


        //o. filter floud
        if(!getGroundPlane()) return;

        //1. create search tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (filtered_cloud_);

        float radius = SEARCH_RADIUS;


        //2. Go through every point, find if centroid is off kilter. If that is the case, keep
        for(pcl::PointCloud<pcl::PointXYZ>::iterator pc_pt_itr  = filtered_cloud_->begin(); 
                                                     pc_pt_itr != filtered_cloud_->end();   ++pc_pt_itr){
            
            pcl::PointXYZ searchPoint = *pc_pt_itr;

            //2. Points that fall within radius, with given distance
            std::vector<int>   pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            
            if (   kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 
               &&  pcl::euclideanDistance(searchPoint, getCentroid(pointIdxRadiusSearch, filtered_cloud_))> CENTROID_DIST){

                edge_cloud_->push_back(searchPoint);
            }
   
        }
        ROS_INFO("[OSM localization]: size of edge_cloud is [%d]", (int)edge_cloud_->size());

        edge_cloud_->is_dense  = true;

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*edge_cloud_, msg);
        msg.header.frame_id = "velodyne";

        filtered_pc_pub_.publish(msg);

        edge_cloud_->clear();


    }




    void OSMLidar::getVelodyneCB(const sensor_msgs::PointCloud2ConstPtr& msg){

        //pcl::PointCloud<pcl::PointXYZI>::Ptr debug_path_points_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *cloud);

        pcl::PCLPointCloud2 pp2;
        pcl::toPCLPointCloud2( *cloud, pp2);

        std::cout << "pp2 size " << pp2.data.size() << std::endl;

        std::cout << msg->header.frame_id << std::endl;

        cloud_deck_.push_front(cloud);
        if(cloud_deck_.size()>= DECK_SIZE) cloud_deck_.pop_back();

    }







}