#include <osm_lidar.h>

namespace osm_localizer{

        uint     OSMLidar::DECK_SIZE       = 7;
        float    OSMLidar::NORM_THRESHOLD  = 0.95; //== cosine(theta)
        period_t OSMLidar::PERIOD          = 4;
        dist_t   OSMLidar::CENTROID_DIST   = 1;
        dist_t   OSMLidar::SEARCH_RADIUS   = 0.3;
        int      OSMLidar::RING_ID          = 0;


    OSMLidar::OSMLidar(ros::NodeHandle & n):
    filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>), 
    edge_cloud_    (new pcl::PointCloud<pcl::PointXYZ>) {

        if(!n.getParam("/osm_localizer/velodyne_topic", pc_topic_)){

            ROS_ERROR("[OSMLidar]: NO velodyne parameter");
            exit(1);
        }
        // 0. subscribers, timers publishers
        pc_sub_          = n.subscribe(pc_topic_, 5, &OSMLidar::getVelodyneCB, this);
        //edge_timer_      = n.createTimer(ros::Duration(PERIOD), &OSMLidar::makeEdges, this);
        road_timer_      = n.createTimer(ros::Duration(0.5), &OSMLidar::filterNonGround, this);
        filtered_pc_pub_ = n.advertise<sensor_msgs::PointCloud2>("chatter", 3);
        ring_pub_        = n.advertise<sensor_msgs::PointCloud2>("rings", 3);

        // 1. dynamic reconfigure
        server_.setCallback(boost::bind(&OSMLidar::reconfigCB, this, _1, _2));

    }


    void OSMLidar::reconfigCB(osm_localization::osm_localizationConfig &config, uint32_t level){

        ROS_INFO_THROTTLE(1.0, "[OSM localization] new parameters requested");
        DECK_SIZE       = config.deck_size;
        NORM_THRESHOLD  = config.norm_threshold; 
        PERIOD          = config.period;
        CENTROID_DIST   = config.centroid_dist;   
        SEARCH_RADIUS   = config.search_radius;     
        RING_ID         = config.ring_id;
        
    }


    void OSMLidar::filterNonGround(const ros::TimerEvent&){

        //0. filter floud
        if(!getGroundPlane()) return; 

        
        //1. Isolate rings  
        makeRings();

        for(const auto & paar : ring_map_){

            std::cout << paar.first << ", ";
        }
        std::cout << std:: endl;

        //2. Get edge points
        /*
        analyzeRings();
        */

        sensor_msgs::PointCloud2 msg;
        if(ring_fanc_map_.find(RING_ID) !=ring_fanc_map_.end()){
            
            pcl::toROSMsg(*ring_fanc_map_[RING_ID], msg);
            msg.header.frame_id = "velodyne";

            ring_pub_.publish(msg);
        }
        

        edge_cloud_->clear();
        filtered_cloud_->clear();
        ring_map_.clear();
        ring_fanc_map_.clear();
    }

    bool ringComp(const std::pair<pcl::PointXYZ, std::size_t> & a, 
                  const std::pair<pcl::PointXYZ, std::size_t> & b){

        return std::atan2(a.first.y, a.first.x) < std::atan2(b.first.y, b.first.x);
    }

    void printVec(std::vector<float> & vec){

        std::cout << "[";
        for(float i : vec){

            std::cout << i  << " ";
        }

        std::cout << "]\n";
    }





    void OSMLidar::makeRings(void){

        //0. Iterate through map and check thetas
        std::size_t N = filtered_cloud_->size();
        std::map <int, std::vector<pcl::PointXYZ>> ring_map;
        std::map <int, pcl::PointCloud<pcl::PointXYZ>::Ptr> ring_fanc_map;


        for(std::size_t i = 0; i < N; ++i){

            pcl::PointXYZ p     = (*filtered_cloud_)[i];
            int           theta = (int)(180*atan2(p.z, sqrt(p.x*p.x + p.y*p.y))/M_PI);

            if(ring_fanc_map.find(theta) ==ring_fanc_map.end()){
                ring_fanc_map[theta] = 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
            ring_map[theta].push_back(p);
            ring_fanc_map[theta]->push_back(p);            
        }

        //1. clump thetas together

        uint ring_number = 0;
        ring_vector_.push_back
            (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));


        std::map<int, std::vector<pcl::PointXYZ>>::iterator i1 = ring_map.begin();
        std::map<int, std::vector<pcl::PointXYZ>>::iterator i2 = i1;

        while(i2!=ring_map_.end()){

            if(std::abs(i2->first- i1->first) >= 3){
                ring_number++;
                ring_vector_.push_back
                    (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));                
            }

            for(pcl::PointXYZ p : i2->second){
                ring_vector_[ring_number]->push_back(p);
                    
            }

            i1 = i2;

            i2++;

            


        }

    }

    /*void OSMLidar::makeRings(void){

        // CLASS STRUCTURES USED IN THIS ALGORITHM 
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  rings_;
        std::vector<std::vector<std::pair<pcl::PointXYZ, std::size_t>>> rings_vec_;
        // END CLASS STRUCTURES USED IN THIS ALGORITHM

        //0. create search tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (filtered_cloud_);
        std::cout << "1. filtered cloud size " << filtered_cloud_->size() << std::endl;
        float radius = SEARCH_RADIUS;

        std::size_t N = filtered_cloud_->size();

        //1. clustering structures
        std::vector<bool>                       visited(filtered_cloud_->size(), false);
        std::size_t                             visited_count  = 0;
        std::size_t                             next_unvisited = 0;
        std::stack<std::size_t>                 frontier;
        std::vector<std::vector<std::size_t>>   clusteres;
        std::size_t                             cluster_id  = 0;

        // 1(a). Initialize frontier, cluster
        frontier.push(0);
        clusteres.push_back(std::vector<std::size_t>());

        //2. Go through every point, 
        while(visited_count < N){
            
            //2(a). We see if we need to start new cluster
            if(frontier.empty()){

                //2(a.i). Find next unvisited
                while(visited[next_unvisited]){
                    next_unvisited++;
                    if(next_unvisited >=N){
                        ROS_ERROR("[OSM Localization] next unvisited bugged up");
                        exit(1);
                    }
                }

                //2(a.ii) Start new cluster
                cluster_id++;
                clusteres.push_back(std::vector<std::size_t>());


                //2(a.iii) Push next unvisited
                frontier.push(next_unvisited);
            }

            //2(b) get next point to find neighors around.
            std::size_t i = frontier.top();
            frontier.pop();
            pcl::PointXYZ searchPoint = (*filtered_cloud_)[i];

            //2(c) Update state & cluster
            if(visited[i]) continue;
            visited[i] = true;
            visited_count++;
            clusteres[cluster_id].push_back(i);
        
            //2(d). Push points that fall within radius to cluster, some to frontier
            std::vector<int>   pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if (kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
                for(std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j){

                    std::size_t k = pointIdxRadiusSearch[j];
                    //2(d.i) if visited, continue
                    if(visited[k]) continue;

                    // 2(d.ii) to frontier
                    if(pointRadiusSquaredDistance[j]>radius/3){
                        frontier.push(k);
                    }

                    // 2(d.iii) directly to cluster
                    else{
                        visited_count++;
                        visited[k]= true;
                        clusteres[cluster_id].push_back(k);
                    }
                }
            }
   
        }

        //3 create rings
        rings_.clear();

        int clust = 0;
        for(const std::vector<std::size_t> & cluster : clusteres){

            rings_.push_back( pcl::PointCloud<pcl::PointXYZ>::Ptr
                                (new pcl::PointCloud<pcl::PointXYZ>));
            rings_vec_.push_back(std::vector<std::pair<pcl::PointXYZ, std::size_t>>());

            for( std::size_t i : cluster){
                rings_[clust]->push_back((*filtered_cloud_)[i]);
                rings_vec_[clust].push_back({(*filtered_cloud_)[i], i});
            }
            rings_[clust]->is_dense = true;
            clust++;
        }
    }
    
    void OSMLidar::analyzeRings(void){

        // 0. Ring by ring, sort by angle, and then calc distance derivatives
        for(std::vector<std::pair<pcl::PointXYZ, std::size_t>> & ring : rings_vec_){

            if(ring.size() == 0 ) continue;

            //1. sort by angle
            std::sort(ring.begin(), ring.end(), ringComp);

            std::vector<float> derivatives(ring.size(), 0);


            //2. Calculate derivatives
            for(int i = 0; i < ring.size()-1; ++i){
                derivatives[i] = pcl::euclideanDistance(ring[i].first, pcl::PointXYZ(0,0,0))-
                                pcl::euclideanDistance(ring[i+1].first, pcl::PointXYZ(0,0,0));

            }


            //printVec(derivatives);


            
        }
    }*/

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

        ROS_INFO("[OSM localization]: size of edge_cloud before process is [%d]", (int)filtered_cloud_->size());

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
        filtered_cloud_->clear();


    }




    void OSMLidar::getVelodyneCB(const sensor_msgs::PointCloud2ConstPtr& msg){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *cloud);

        pcl::PCLPointCloud2 pp2;
        pcl::toPCLPointCloud2( *cloud, pp2);

        std::cout << "pp2 size " << pp2.data.size() << std::endl;


        cloud_deck_.push_front(cloud);
        if(cloud_deck_.size()>  DECK_SIZE) cloud_deck_.pop_back();

    }







}