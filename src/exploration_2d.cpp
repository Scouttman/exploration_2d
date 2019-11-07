// Related headers:
#include "exploration_2d.h"
#include "navigation_utils.h"
#include "gpregressor.h"
#include "covMaterniso3.h"

//C library headers:
#include <iostream>
#include <fstream>
#include <string>
// #include <chrono>
// #include <iterator>
// #include <ctime>

//C++ library headers:  NONE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//other library headers:  NONE

//Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <time.h>
//
//// Cost Map 
//#include <costmap_2d/costmap_2d_ros.h>
//#include "grid_layer.h"
//
//// navfs
//#include <navfn/navfn_ros.h>
//#include <navfn/navfn.h>
//
//#include "tf2_ros/buffer.h"
//#include "tf2_ros/transform_listener.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "costmap_client.h"
#include "occupancy_grid_a_star.cpp"



using namespace std;
using namespace grid_map;


int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot_exploration_3d");
    ros::NodeHandle nh;

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);

    std::fstream logfile;
    logfile.open ("log.txt",std::ios::out | std::ios::app);

    //strftime(buffer,80,"Octomap2D_%m%d_%R.ot",timeinfo);
    //octomap_name_2d = buffer;

    //ros::Subscriber kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, kinectCallbacks);// need to change##########
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);
    ros::Subscriber polygon_sub = nh.subscribe("/clicked_point", 1, pointCb);
    //PolygonBounds(nh);
    //click_server::PolygonBounds client;
    //client.pointCb(nh);
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "/Goal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("/Candidate_MIs", 1);
    ros::Publisher Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("/Frontier_points", 1);
    Hot_points_pub = nh.advertise<visualization_msgs::Marker>("/hot_spots", 1);
    //ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //ros::Publisher Octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_3d",1);
    ros::Publisher Octomap_flat_pub = nh.advertise<octomap_msgs::Octomap>("octomap_2d",1);
    ros::Publisher Octomap_low_pub = nh.advertise<octomap_msgs::Octomap>("octomap_2d_low",1);
    // TODO cost map 
    // seems to publish an occupancy grid  /move_base/my_costmap/costmap
    // publishes and update occpancy grid as well
    // not topics to subscribe to ??
    // move base is owner 
    ros::Subscriber costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",1,costmapCb);

    // Initialize node and publisher.
    //ros::init(argc, argv, "grid_map_simple_demo");
    ros::NodeHandle nhGrid("~");
    ros::Publisher publisher = nhGrid.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Publisher pub_grid_bay = nhGrid.advertise<grid_map_msgs::GridMap>("grid_map_bay", 1, true);
    //ros::NodeHandle private_node_handle("~");
    nhGrid.param<bool>("normalise", normalise, true);          // Normalise base on distance
    nhGrid.param<float>("norm_fac",normalising_factor,0.7);      // Strength of normalisation
    nhGrid.param<bool>("momentum", preserve_momentum, false);   // try to keep heading
    nhGrid.param<float>("head_amp",head_amp,10);                // Strength of heading
    nhGrid.param<int>("samples_eva", num_of_samples_eva,50);    
    nhGrid.param<bool>("run_once", run_once, false);
    nhGrid.param<bool>("continuous",continuous, false);
    nhGrid.param<bool>("simple_weight",simple_weight,true);   
    nhGrid.param<bool>("wait_for_bounds",wait_for_bounds,false); 
    nhGrid.param<float>("point_weight",point_weight,0.01);   



    // Create grid map.
    GridMap map({"elevation"});
    map.setFrameId("map");
    //map.setGeometry(Length(1.2, 2.0), 0.03);
    map.setGeometry(Length(8.0, 8.0), 0.1, Position(0.0, -0.1));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));
    srand (time(NULL));

    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading; // robot's heading direction
    
    
    /*######################### Cost Map Stuff  ##########################*/
    
    
    // From move_base.cpp
    //ros::NodeHandle private_nh("~");
    //std::string robot_base_frame_, global_frame_;
    //boost::thread* planner_thread_;
    //costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;
    //private_nh.param("my_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    //private_nh.param("my_costmap/global_frame", global_frame_, std::string("map"));
    //
    ////set up the planner's thread
    ////planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));
    //// TF stuff
    //tf2_ros::Buffer tf_buffer;
    //tf2_ros::TransformListener tfl(tf_buffer);
    ////costMap_tf.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
    //geometry_msgs::TransformStamped tfGeom;
    //try {
    //    tfGeom = tf_buffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
    //} catch (tf2::TransformException &e) {
    //    // handle lookup error
    //}
    //
    ////explore::Costmap2DClient costmap_new = new explore::Costmap2DClient(private_nh, nh, tf_listener);
    ////explore::Costmap2DClient costmap_new(private_nh, nh, tf_listener);
    ////search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),potential_scale_, gain_scale_,min_frontier_size);
    ////create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    //planner_costmap_ros_ = new costmap_2d::Costmap2DROS("my_costmap", tf_buffer);
    //planner_costmap_ros_->pause();
    //
    ////initialize the global planner
    //navfn::NavfnROS navfn;
    //navfn.initialize("my_navfn_planner", planner_costmap_ros_);
    ////try {
    //    //planner_ = bgp_loader_.createInstance(global_planner);
    //    //planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    ////} catch (const pluginlib::PluginlibException& ex) {
    ////  ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
    ////  exit(1);
    ////}
    //
    //planner_costmap_ros_->start();
    //
    ////simple_layer_namespace::SimpleLayer simple_costmap;

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker Frontier_points_cubelist;
    geometry_msgs::Twist twist_cmd;

    ros::Time now_marker = ros::Time::now();
   
    // Initialize parameters 
    int max_idx = 0;

    octomap::OcTreeNode *n;
    octomap::OcTree new_tree_flat(octo_reso);
    cur_tree_flat = &new_tree_flat;
    octomap::OcTree new_tree_low(octo_reso_low);
    cur_tree_low = &new_tree_low;
    octomap::OcTree new_tree_info(octo_reso);
    tree_info = &new_tree_info;
    point3d next_vp;

    bool got_tf = false;
    bool arrived;

    // Generate simple weights
    // insert some measurements of occupied cells
    // http://docs.ros.org/api/octomap/html/simple__example_8cpp_source.html
    float max_dis = hypot(map_length,map_width);
    for (float x=-map_width; x<map_width; x+=step_size) {
        for (float y=-map_length; y<map_length; y+=step_size) {
            point3d endpoint (x, y, 0);
            octomap::OcTreeNode* node = tree_info->updateNode(endpoint, true); // integrate 'occupied' measurement
            tree_info->updateNodeLogOdds(node, hypot(x,y)/max_dis); //Insert value
        }
    }


    // Get the points for weights
    if(wait_for_bounds){
        ROS_INFO("Waiting For Boundary");
        while(!kbhit() && ros::ok())
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
    
    ROS_INFO("Points");
    if (input_.polygon.points.size() > 1){
        for (unsigned int i = 0; i < input_.polygon.points.size(); i++)
        {
            ROS_INFO("point:%f,%f",input_.polygon.points[i].x,input_.polygon.points[i].y);
        }
    }
    wait_for_bounds = false;
    
    // Update the initial location of the robot
    got_tf = false;
    while(!got_tf && ros::ok()){
        try{
            tf_listener->lookupTransform("/map", "/base_scan", ros::Time(0), transform);// need to change tf of kinect###############
            kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            kinect_head = tf::getYaw(transform.getRotation());
            ROS_INFO("Kinect heading %f",kinect_head);
            got_tf = true;
            logfile << ros::Time::now().toSec()<< "," << kinect_orig.x() << "," << kinect_orig.y() <<"\n";
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: Kinect frame"); 
        }
        ros::Duration(0.05).sleep();
        ros::Time now = ros::Time::now();
        tf_listener->waitForTransform("/map", "/base_scan", now, ros::Duration(1.0));
    }

    ROS_INFO("Taking a scan");
    // Take a Scan
    ros::spinOnce();

    // prepare octomap msg
    octomap_msgs::binaryMapToMsg(*cur_tree_flat, msg_octomap_flat);
    msg_octomap_flat.binary = 1;
    msg_octomap_flat.id = 1;
    msg_octomap_flat.resolution = octo_reso;
    msg_octomap_flat.header.frame_id = "/map";
    msg_octomap_flat.header.stamp = ros::Time::now();
    Octomap_flat_pub.publish(msg_octomap_flat);

    octomap_msgs::binaryMapToMsg(*cur_tree_low, msg_octomap_low);
    msg_octomap_low.binary = 1;
    msg_octomap_low.id = 2;
    msg_octomap_low.resolution = octo_reso_low;
    msg_octomap_low.header.frame_id = "/map";
    msg_octomap_low.header.stamp = ros::Time::now();
    Octomap_low_pub.publish(msg_octomap_low);

    // steps robot taken, counter
    int robot_step_counter = 0;

    ROS_INFO("Startup Compltete");

    //////////////////////////////////////////////////// Main Loop /////////////////////////////////////////////////////////////////////////
    while (ros::ok())
    {
        //ROS_INFO("octopmap vol ? %lu",countVolume(cur_tree_flat));
        //vector<vector<point3d>> frontier_groups=extractFrontierPoints(cur_tree_flat);
        vector<vector<point3d>> frontier_groups=extractFrontierPoints(cur_tree_low);
        if(frontier_groups.size()==0)
            ROS_WARN("No Current Frontier Points");
        
        //frontier_groups.clear();//in the next line
        unsigned long int o = 0;
        for(vector<vector<point3d>>::size_type e = 0; e < frontier_groups.size(); e++) {
            o = o+frontier_groups[e].size();
        }

        Frontier_points_cubelist.points.resize(o);
        Frontier_points_cubelist.header.frame_id = "map";
        Frontier_points_cubelist.header.stamp = ros::Time::now();
        Frontier_points_cubelist.ns = "frontier_points_array";
        Frontier_points_cubelist.id = 0;
        Frontier_points_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
        Frontier_points_cubelist.action = visualization_msgs::Marker::ADD;
        Frontier_points_cubelist.scale.x = octo_reso;
        Frontier_points_cubelist.scale.y = octo_reso;
        Frontier_points_cubelist.scale.z = octo_reso;
        Frontier_points_cubelist.color.a = 1.0;
        Frontier_points_cubelist.color.r = (double)255/255;
        Frontier_points_cubelist.color.g = 0;
        Frontier_points_cubelist.color.b = (double)0/255;
        Frontier_points_cubelist.lifetime = ros::Duration();

        unsigned long int t = 0;
        int l = 0;
        geometry_msgs::Point q;
        for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++) { 
            for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
               q.x = frontier_groups[n][m].x();
               q.y = frontier_groups[n][m].y();
               q.z = frontier_groups[n][m].z()+octo_reso;
               Frontier_points_cubelist.points.push_back(q); 
            }
            t++;
        }
        
        Frontier_points_pub.publish(Frontier_points_cubelist); //publish frontier_points
        Frontier_points_cubelist.points.clear();           

        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = extractCandidateViewPoints(frontier_groups, kinect_orig, 50); 
        std::random_shuffle(candidates.begin(),candidates.end()); // shuffle to select a subset
        vector<pair<point3d, point3d>> gp_test_poses = candidates;
        ROS_INFO("Candidate View Points: %lu Genereated, %d evaluating...", candidates.size(), num_of_samples_eva);
        int temp_size = candidates.size()-3;
        if (temp_size < 1) {
            ROS_ERROR("Very few candidates generated, finishing with exploration...");
            nh.shutdown();
            nhGrid.shutdown();
            return 0;
        }

        // Generate Testing poses
        candidates.resize(min(num_of_samples_eva,temp_size));
        frontier_groups.clear();

        // Evaluate MI for every candidate view points
        vector<double>  MIs(candidates.size());
        double before = countFreeVolume(cur_tree_flat);
        int max_idx = 0;
        int min_idx = 0;
        double miRange;
        double begin_mi_eva_secs, end_mi_eva_secs;
        begin_mi_eva_secs = ros::Time::now().toSec();

        float goal_dis;
        float goal_dis_new;
        float goal_head;
        float head_offset;
        float point_dis;
        octomap::OcTreeNode* result;
        /// ######################################################### My STUFF ###############################################################
        //navfn.setStart();
        printf("My Stuff");
        geometry_msgs::PoseStamped start;
        //geometry_msgs::Point start;
        start.pose.position.x = kinect_orig.x();
        start.pose.position.y = kinect_orig.y();
        start.header.frame_id = "map";
        start.header.stamp = ros::Time::now();
        //start..x = kinect_orig.x();
        //start..y = kinect_orig.y();
        geometry_msgs::PoseStamped goal;
        std::vector<geometry_msgs::PoseStamped> plan;
        std::vector<geometry_msgs::Point> points_path;


        printf("headings?:");
        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            // Cast rays at candidate sensor position to see what objects hit??
            octomap::Pointcloud hits = castSensorRays(cur_tree_flat, c.first, c.second);
            
            
            // Goal dis head
            goal_dis = hypot(c.first.x()-kinect_orig.x(),c.first.y()-kinect_orig.y());
            goal.pose.position.x = c.first.x();
            goal.pose.position.y = c.first.y();
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            //navfn.makePlan(start,goal,plan);
            
            const auto path = path_planning::a_star(occupancy_grid_in,start.pose.position,goal.pose.position);
            float goal_dis_new = 0;
            if(path){
                ROS_INFO("Path len %lu",path->size());
                for(int t =1; t<path->size(); t++){
                    goal_dis_new+= hypot(path->at(t-1).x-path->at(t).x,path->at(t-1).y-path->at(t).y);
                }
                //for(const auto &p : path)
                //{
                //    ROS_INFO("%f",(*p).x);
                //}
                //for(geometry_msgs::Point i : path){
                //    ROS_INFO("%f,%f",i.x,i.y);
                //} 
            }
            
            //goal_dis_new = navfn.getPathLen();
            ROS_INFO("goaldis new %f \told %f",goal_dis_new,goal_dis);
            goal_dis = goal_dis_new;
            //ROS_INFO_THROTTLE(1.0,"goaldis new %f \told %f",goal_dis_new,goal_dis);
            //int goal [2] = {(c.first.x()-costmap->origin_x)/costmap->resolution,(c.first.y()-costmap.resolution)/costmap.resolution};
            //navfn.setGoal(*goal);
            //if(navfn.calcNavFnAstart()){
            //    goal_dis = navfn.getPathLen();
            //}
            goal_head = atan2(c.first.y()-kinect_orig.y(),c.first.x()-kinect_orig.x());
            // TODO all MIS are very similar??
            // Evaluate Mutual Information
            if(normalise){
                // Normalize the MI with distance
                // TODO try normalsing based on A* distance?? //checkout the occupancy_grid_a_star
                //MIs[i] = calc_MI(cur_tree_flat, c.first, hits, before) / 
                //    (goal_dis);
                //float result = simple_cal(cur_tree_flat, c.first, c.second);
                float eval_MI = calc_MI(cur_tree_flat, c.first, hits, before);
                float gain = 0.5;
                // My be don't normalise if the point is not within line of sight 
                MIs[i] = normalising_factor*(eval_MI/goal_dis)+(1-normalising_factor)*eval_MI;
            }else{
                MIs[i] = calc_MI(cur_tree_flat, c.first, hits, before);
            }
            // prefer movemnt in the current direction
            if(preserve_momentum){
                // get heading abs offset
                head_offset = abs(goal_head-kinect_head);
                if(head_offset>M_PI)
                    head_offset = 2*M_PI-head_offset;
                MIs[i] += (M_PI-head_offset)*head_amp; // Invert and multiply //TODO +=
                //printf("%f,",(head_offset)*head_amp);

            }
            //if(simple_weight){
            //    //Round to cell size?
            //    grid_space = 1/step_size;
            //    //ROS_INFO("Grid space %f",grid_space);
            //    point3d query (floorf(c.first.x() * grid_space) / grid_space,floorf(c.first.y() * grid_space) / grid_space,0);
            //    result = tree_info->search (query);
            //    //print_query_info(query, result);
            //    if (result != NULL)
            //        MIs[i] += result->getOccupancy()/10;
            //}

            if(simple_weight){
                if (input_.polygon.points.size() > 1){
                    for (unsigned int e = 0; e < input_.polygon.points.size(); e++)
                    {
                        point_dis = hypot(input_.polygon.points[e].x-c.first.x(),input_.polygon.points[e].y-c.first.y());
                        if(point_dis < 1){
                            MIs[i]=0; // Don't go
                            break;
                        }else{
                            if(point_dis > 5)
                                point_dis = 5; // limit range
                            MIs[i] -= point_dis*point_weight;
                        }
                    }
                }
            }
            if(MIs[i]<0)
                MIs[i] = 0;

            //Pick the Candidate view point with max MI
            if (MIs[i] > MIs[max_idx])
                max_idx = i;
            if (MIs[i] < MIs[min_idx])
                min_idx = i;
        }
        printf("\n");
        if(MIs[max_idx] == MIs[min_idx])
            ROS_WARN("MAX and MIN MI the same");
        if(MIs[max_idx] < MIs[min_idx]+0.1)
            ROS_WARN("MAX and MIN MI %f appart",MIs[max_idx]-MIs[min_idx]);
        else
            ROS_INFO("MAX and MIN MI %f appart",MIs[max_idx]-MIs[min_idx]);
        ROS_INFO("Max MI %f Min MI %f",MIs[max_idx],MIs[min_idx]);

        // Add data to grid map.
        ros_time = ros::Time::now();
        map.setGeometry(Length(5.0, 5.0), 0.1, Position(transform.getOrigin().x(), transform.getOrigin().y()));
        miRange = MIs[max_idx]-MIs[min_idx];
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            Position position = Position(c.first.x(), c.first.y());
            if (map.isInside(position))
                map.atPosition("elevation",position) = 0.1*(double)(MIs[i]-MIs[min_idx])/miRange; //+ static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        }
        // Publish grid map.
        map.setTimestamp(ros_time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        map.clearAll();

        // Bayesian Optimization for actively selecting candidate
        double train_time, test_time;
        GPRegressor g(100, 3, 0.01);
        for (int bay_itr = 0; bay_itr < num_of_bay; bay_itr++) {
            //Initialize gp regression
            
            MatrixXf gp_train_x(candidates.size(), 3), gp_train_label(candidates.size(), 1), gp_test_x(gp_test_poses.size(), 3);

            for (int i=0; i< candidates.size(); i++){
                gp_train_x(i,0) = candidates[i].first.x();
                gp_train_x(i,1) = candidates[i].first.y();
                gp_train_x(i,2) = candidates[i].second.z();
                gp_train_label(i) = MIs[i];
            }

            for (int i=0; i< gp_test_poses.size(); i++){
                gp_test_x(i,0) = gp_test_poses[i].first.x();
                gp_test_x(i,1) = gp_test_poses[i].first.y();
                gp_test_x(i,2) = gp_test_poses[i].second.z();
            }

            // Perform GP regression
            MatrixXf gp_mean_MI, gp_var_MI;
            train_time = ros::Time::now().toSec();
            g.train(gp_train_x, gp_train_label);
            train_time = ros::Time::now().toSec() - train_time;

            test_time = ros::Time::now().toSec();
            g.test(gp_test_x, gp_mean_MI, gp_var_MI);
            test_time = ros::Time::now().toSec() - test_time;

            // Get Acquisition function
            double beta = 2.4;
            vector<double>  bay_acq_fun(gp_test_poses.size());
            for (int i = 0; i < gp_test_poses.size(); i++) {
                bay_acq_fun[i] = gp_mean_MI(i) + beta*gp_var_MI(i);
            }
            vector<int> idx_acq = sort_MIs(bay_acq_fun);

            // evaluate MI, add to the candidate
            auto c = gp_test_poses[idx_acq[0]];
            octomap::Pointcloud hits = castSensorRays(cur_tree_flat, c.first, c.second);
            candidates.push_back(c);
            MIs.push_back(calc_MI(cur_tree_flat, c.first, hits, before));
            gp_test_poses.erase(gp_test_poses.begin()+idx_acq[0]);
        }
        
        end_mi_eva_secs = ros::Time::now().toSec();
        ROS_INFO("Mutual Infomation Eva took:  %3.3f Secs.", end_mi_eva_secs - begin_mi_eva_secs);

        // This was already commented out
        // Normalize the MI with distance
        // for(int i = 0; i < candidates.size(); i++) {
        //     auto c = candidates[i];
        //     MIs[i] = MIs[i] / 
        //         sqrt(pow(c.first.x()-kinect_orig.x(),2) + pow(c.first.y()-kinect_orig.y(),2));
        // }

        // sort vector MIs, with idx_MI, descending
        vector<int> idx_MI = sort_MIs(MIs);
        if (MIs[0] < 0.00) {
            ROS_ERROR("Highest MI %f exiting...",MIs[0]);
            nh.shutdown();
            nhGrid.shutdown();
            return 0;
        }

        // Publish the candidates as marker array in rviz
        tf::Quaternion MI_heading;
        MI_heading.setRPY(0.0, -PI/2, 0.0);
        MI_heading.normalize();
        std::string miArray; 
        double minMI = MIs[idx_MI.back()];
        double diviser = MIs[idx_MI[0]]-minMI;
        double miRel, posRel;
        CandidatesMarker_array.markers.resize(candidates.size());
        for (int i = 0; i < candidates.size(); i++)
        {
            miRel = (double)(MIs[i]-minMI)/diviser;
            posRel = ((double)i)/candidates.size();
            CandidatesMarker_array.markers[i].header.frame_id = "map";
            CandidatesMarker_array.markers[i].header.stamp = ros::Time::now();
            CandidatesMarker_array.markers[i].ns = "candidates";
            CandidatesMarker_array.markers[i].id = i;
            CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
            CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
            CandidatesMarker_array.markers[i].pose.position.x = candidates[i].first.x();
            CandidatesMarker_array.markers[i].pose.position.y = candidates[i].first.y();
            CandidatesMarker_array.markers[i].pose.position.z = candidates[i].first.z();
            CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
            CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
            CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
            CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
            CandidatesMarker_array.markers[i].scale.x = 0.5+(double)0.5*miRel;//(MIs[i]-minMI)/diviser;
            CandidatesMarker_array.markers[i].scale.y = 0.05;
            CandidatesMarker_array.markers[i].scale.z = 0.05;
            if(MIs[i]<=0.01)
                CandidatesMarker_array.markers[i].color.a = 0.3;
            else
                CandidatesMarker_array.markers[i].color.a = 1; //(double)(MIs[i]-minMI)/diviser;
            //CandidatesMarker_array.markers[i].color.r = 1.0;
            //CandidatesMarker_array.markers[i].color.g = 0.55;
            //CandidatesMarker_array.markers[i].color.b = 0.22;
            CandidatesMarker_array.markers[i].color.r = toHSV(miRel);
            CandidatesMarker_array.markers[i].color.g = toHSV(miRel+1.0/3.0);
            CandidatesMarker_array.markers[i].color.b = toHSV(miRel+2.0/3.0);
            miArray.append(std::to_string(MIs[i]));//MIs[i]-minMI)/diviser
            miArray.append(",");
        }
        // Add data to grid map.
        ros_time = ros::Time::now();
        map.setGeometry(Length(5.0, 5.0), 0.1, Position(transform.getOrigin().x(), transform.getOrigin().y()));
        min_idx = idx_MI.back();
        max_idx = idx_MI[0];
        miRange = MIs[max_idx]-MIs[min_idx];
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            Position position = Position(c.first.x(), c.first.y());
            if (map.isInside(position))
                map.atPosition("elevation",position) = 0.1*(double)(MIs[i]-MIs[min_idx])/miRange; //+ static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        }

        // Publish grid map.
        map.setTimestamp(ros_time.toNSec());
        GridMapRosConverter::toMessage(map, message);
        pub_grid_bay.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        map.clearAll();

        //ROS_INFO("MI %s",miArray);
        std::cout << "MI: " << miArray << endl;
        ROS_INFO("MI DIFF %f",MIs[idx_MI[0]]-minMI);
        Candidates_pub.publish(CandidatesMarker_array);
        CandidatesMarker_array.markers.clear();
        candidates.clear(); //TODO all candidates delted?? 

        arrived = false;
        int idx_ptr = 0;
        if(run_once){
            nh.shutdown();
            nhGrid.shutdown();
            return 0;
        }
            

        while (!arrived && ros::ok()) {
            // Setup the Goal
            next_vp = point3d(candidates[idx_MI[idx_ptr]].first.x(),candidates[idx_MI[idx_ptr]].first.y(),0); //candidates[idx_MI[idx_ptr]].first.z());
            logfile << ros::Time::now().toSec() << "," << kinect_orig.x() << "," << kinect_orig.y() <<"\n";
            // Get latest heading
            tf_listener->lookupTransform("/map", "/base_scan", ros::Time(0), transform);
            kinect_head = tf::getYaw(transform.getRotation()); // update robot heading
            Goal_heading.setRPY(0.0, 0.0, kinect_head); // Set heading to the direction of travel
            //Goal_heading.setRPY(0.0, 0.0, candidates[idx_MI[idx_ptr]].second.yaw());
            Goal_heading.normalize();
            ROS_INFO("Max MI : %f , @ location: %3.2f  %3.2f", MIs[idx_MI[idx_ptr]], next_vp.x(), next_vp.y() );

            // Publish the goal as a Marker in rviz
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "goal_marker";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = next_vp.x();
            marker.pose.position.y = next_vp.y();
            marker.pose.position.z = 1.0;
            marker.pose.orientation.x = Goal_heading.x();
            marker.pose.orientation.y = Goal_heading.y();
            marker.pose.orientation.z = Goal_heading.z();
            marker.pose.orientation.w = Goal_heading.w();
            marker.scale.x = 0.5;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            GoalMarker_pub.publish( marker );

            // Send the Robot 
            if(!continuous){
                //arrived = goToDest(next_vp, Goal_heading);
                arrived = goToDestEarly(next_vp, Goal_heading,0.5);
            }else{
                arrived = goToDestEarly(next_vp, Goal_heading,5);  // TODO try setting arrived to true for continuous running 
            }

            if(arrived)
            {
                // Update the initial location of the robot
                got_tf = false;
                while(!got_tf){
                    try{
                        tf_listener->lookupTransform("/map", "/base_scan", ros::Time(0), transform);
                        kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                        kinect_head = tf::getYaw(transform.getRotation()); // update robot heading
                        got_tf = true; 
                    }
                    catch (tf::TransformException ex) {
                        ROS_WARN("Wait for tf: base_scan"); 
                    } 
                    ros::Duration(0.05).sleep();
                    ros::Time now = ros::Time::now();
                    tf_listener->waitForTransform("/map", "/base_scan", now, ros::Duration(3.0));
                }
                // Update Octomap
                ros::spinOnce();
                ROS_INFO("Succeed, new Map Free Volume: %f", countFreeVolume(cur_tree_flat));
                robot_step_counter++;

                octomap_msgs::binaryMapToMsg(*cur_tree_flat, msg_octomap_flat);
                msg_octomap_flat.binary = 1;
                msg_octomap_flat.id = 1;
                msg_octomap_flat.resolution = octo_reso;
                msg_octomap_flat.header.frame_id = "/map";
                msg_octomap_flat.header.stamp = ros::Time::now();
                Octomap_flat_pub.publish(msg_octomap_flat);

                octomap_msgs::binaryMapToMsg(*cur_tree_low, msg_octomap_low);
                msg_octomap_low.binary = 1;
                msg_octomap_low.id = 2;
                msg_octomap_low.resolution = octo_reso_low;
                msg_octomap_low.header.frame_id = "/map";
                msg_octomap_low.header.stamp = ros::Time::now();
                Octomap_low_pub.publish(msg_octomap_low);

                //Shurelly I need to remove the MI after evaulation ??
                MIs.erase(MIs.begin()+idx_MI[idx_ptr]);
            }
            else
            {
                ROS_WARN("Failed to drive to the %d th goal, switch to the sub-optimal..", idx_ptr);
                idx_ptr++;
                if(idx_ptr > MIs.size()) {
                    ROS_ERROR("None of the goal is valid for path planning, shuting down the node");
                    nh.shutdown();
                    nhGrid.shutdown();
                }
            }

        } 
        // r.sleep();
    }
    nh.shutdown();
    nhGrid.shutdown();
    logfile.close();   
    return 0;
}