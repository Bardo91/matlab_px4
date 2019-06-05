//
//
//
//
//
//
//


#include <TrajectoryVisualizer.h>
#include <geometry_msgs/PoseStamped.h>

//-------------------------------------------------------------------------------------------------------------
TrajectoryVisualizer::TrajectoryVisualizer(){
        viewer_ = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("motion_planner_visualizer"));
        viewer_->addCoordinateSystem(0.5);

        ros::NodeHandle nh;
        trajectorySubscriber_ = nh.subscribe("/emanuela/computed_trajectory", 1, &TrajectoryVisualizer::trajectoryCallback, this);
        waypointsSubscriber_ = nh.subscribe("/emanuela/waypoints", 1, &TrajectoryVisualizer::waypointsCallback, this);
}

//-------------------------------------------------------------------------------------------------------------
void TrajectoryVisualizer::draw(const std::vector<Eigen::Vector3f> &_trajectory, unsigned char _r, unsigned char _g, unsigned char _b, bool _useSpline){
        // Create new graph
        vtkSmartPointer<vtkPolyData> covisibilityGraph = vtkSmartPointer<vtkPolyData>::New();
        covisibilityGraph->Allocate();
        trajectories_.push_back(covisibilityGraph);
    
        vtkSmartPointer<vtkUnsignedCharArray> covisibilityNodeColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        covisibilityNodeColors->SetNumberOfComponents(3);
        covisibilityNodeColors->SetName("Colors");

        vtkSmartPointer<vtkPoints> covisibilityNodes = vtkSmartPointer<vtkPoints>::New();

        // Fill-up with nodes
        auto &points = _trajectory;
        
        if(_useSpline){
            int splineFactor = _trajectory.size() > 5 ? 5 : _trajectory.size(); 
            Spline<Eigen::Vector3f, float> spl(splineFactor);
            spl.set_ctrl_points(points);
            int nPoints = points.size()*10;
            for(unsigned i = 0; i < nPoints; i++){
                auto p = spl.eval_f(1.0 / nPoints* i );
                const unsigned char green[3] = {_r, _g, _b};
                covisibilityNodes->InsertNextPoint(     p[0], 
                                                        p[1], 
                                                        p[2]);
                covisibilityNodeColors->InsertNextTupleValue(green);
                if(i > 0){
                    vtkIdType connectivity[2];
                    connectivity[0] = i-1;
                    connectivity[1] = i;
                    covisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity);
                }
            }
        }else{
            for(unsigned i = 0; i <  points.size(); i++){
                const unsigned char green[3] = {_r, _g, _b};
                covisibilityNodes->InsertNextPoint(    points[i][0], 
                                                        points[i][1], 
                                                        points[i][2]);
                covisibilityNodeColors->InsertNextTupleValue(green);
                if(i > 0){
                    vtkIdType connectivity[2];
                    connectivity[0] = i-1;
                    connectivity[1] = i;
                    covisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity);
                }
            }            
        }
        covisibilityGraph->SetPoints(covisibilityNodes);
        covisibilityGraph->GetPointData()->SetScalars(covisibilityNodeColors);
        
        viewer_->addModelFromPolyData(covisibilityGraph, "traj_"+std::to_string(itemCounter_));
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "traj_"+std::to_string(itemCounter_));
        
        itemCounter_++;
}

//-------------------------------------------------------------------------------------------------------------
void TrajectoryVisualizer::trajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    std::cout << "Arrived trayectory \n";    
    std::cout << "computed trajectory size " << msg->data.size() << "\n";
    std::vector<Eigen::Vector3f> traj;
    for(unsigned i = 0; i < msg->data.size(); i = i + 6){
        Eigen::Vector3f wp = {  msg->data[i],
                                msg->data[i+1],
                                msg->data[i+2]};
            
        traj.push_back(wp);
    }
    std::cout << "trajectory size size " <<traj.size() << "\n";
    int r = rangeRandom(0,255);
    int g = rangeRandom(0,255);
    int b = rangeRandom(0,255);
    this->draw(traj,r,g,b,false);
    this->drawSphere(traj[0],0.1);
}

//-------------------------------------------------------------------------------------------------------------
void TrajectoryVisualizer::waypointsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    std::cout << "Arrived waypoints \n";    
    std::cout << "waypoints size " << msg->data.size() << "\n";
    std::vector<Eigen::Vector3f> traj;
    for(unsigned i = 0; i < msg->data.size(); i = i + 9){
        Eigen::Vector3f wp = {  msg->data[i],
                                msg->data[i+1],
                                msg->data[i+2]};
        this->drawSphere(wp,0.3);
        traj.push_back(wp);
    }

    std::cout << "waypoints size size " <<traj.size() << "\n";
    this->draw(traj, 255, 0, 0, false);
}

//-------------------------------------------------------------------------------------------------------------
void TrajectoryVisualizer::drawSphere(const Eigen::Vector3f &_center, float _radius){
    std::string itemName = "sphere_"+std::to_string(itemCounter_);
    viewer_->addSphere(pcl::PointXYZ(_center[0], _center[1], _center[2]), _radius, itemName);

    itemCounter_++;
} 

//-------------------------------------------------------------------------------------------------------------
void TrajectoryVisualizer::spin(){
    viewer_->spin();
}

//-------------------------------------------------------------------------------------------------------------
void TrajectoryVisualizer::spinOnce(){
    viewer_->spinOnce();
}

//-------------------------------------------------------------------------------------------------------------

int TrajectoryVisualizer::rangeRandom(int _min, int _max){
    int n = _max - _min + 1;
    int remainder = RAND_MAX % n;
    int x;
    do{
        x = rand();
    }while (x >= RAND_MAX - remainder);

    return _min + x % n;
}