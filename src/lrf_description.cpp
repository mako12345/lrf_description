

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#define PI 3.14159265

class lrf_description
{
        //関数宣言
    public:
        lrf_description();
        ~lrf_description();
		
		void scan_callback(const sensor_msgs::LaserScan &sensor_msg);
		
        //変数宣言
    private:
        ros::Timer timer;
        ros::NodeHandle n;
        ros::Publisher marker_pub;
        
        tf::TransformBroadcaster tf_pub_;
        
        ros::Subscriber scan_sub;
        geometry_msgs::Vector3 pos_wall[512];
        
        geometry_msgs::Vector3 hsv_value;
        geometry_msgs::Vector3 rgb_value;
        
        
        double wall_high = 0.5;
        double angle_range;
        double alpha = 0;
        
        float free_space_high[512];
        tf::StampedTransform free_base_tf[512];
        
        
        
        
        
        
};

std::string value2string(float value)
{
	std::stringstream ss;
	std::string stringout;
	
	ss << value;
	stringout = ss.str();
	return stringout;
}


geometry_msgs::Vector3 hsv2rgb(geometry_msgs::Vector3 hsv)
{
    geometry_msgs::Vector3  rgb;
    int     Hi;
    float   f;
    float   p;
    float   q;
    float   t;

    Hi = fmod(floor(hsv.x / 60.0f), 6.0f);
    f = hsv.x / 60.0f - Hi;
    p = hsv.z * (1.0f - hsv.y);
    q = hsv.z * (1.0f - f * hsv.y);
    t = hsv.z * (1.0f - (1.0f - f) * hsv.y);

    if(Hi == 0){
        rgb.x = hsv.z;
        rgb.y = t;
        rgb.z = p;
    }
    if(Hi == 1){
        rgb.x = q;
        rgb.y = hsv.z;
        rgb.z = p;
    }
    if(Hi == 2){
        rgb.x = p;
        rgb.y = hsv.z;
        rgb.z = t;
    }
    if(Hi == 3){
        rgb.x = p;
        rgb.y = q;
        rgb.z = hsv.z;
    }
    if(Hi == 4){
        rgb.x = t;
        rgb.y = p;
        rgb.z = hsv.z;
    }
    if(Hi == 5){
        rgb.x = hsv.z;
        rgb.y = p;
        rgb.z = q;
    }

    return rgb;
}

void lrf_description::scan_callback(const sensor_msgs::LaserScan &sensor_msg)
{
	
	visualization_msgs::MarkerArray marker;
	marker.markers.resize(1024);
		
	for(int i = 0; i < 512; i++)
	{
		//if(sensor_msg.ranges[i] < 2){ 
			
		//}
		
		
		if(isinf(sensor_msg.ranges[i]))
		{
			alpha = 0;
			pos_wall[i].x = 0;
			pos_wall[i].y = 0;
		}
		else
		{
			
			pos_wall[i].x = sensor_msg.ranges[i] * cos(((3 * PI / 2) / 750) * i - (PI / 4) - (PI / 2) + (PI / 4));
			pos_wall[i].y = sensor_msg.ranges[i] * sin(((3 * PI / 2) / 750) * i - (PI / 4) - (PI / 2) + (PI / 4));
			
			alpha = 1;
			
			
		}
		
		
		
		if(sensor_msg.ranges[i] < 0.5)
		{
			hsv_value.x = 1;
		}
		
		else if(sensor_msg.ranges[i] < 3)
		{
			hsv_value.x = (sensor_msg.ranges[i] - 0.5) * 250 / 3;
		}
		else 
		{
			hsv_value.x = 250;
		}
		hsv_value.y = 1;
		hsv_value.z = 1;
		
		rgb_value = hsv2rgb(hsv_value);
		
		
		marker.markers[i].header.frame_id = "laser";
		marker.markers[i].header.stamp = ros::Time();
		marker.markers[i].ns = "wall";
		marker.markers[i].id = i;
		marker.markers[i].type = visualization_msgs::Marker::CUBE;
		marker.markers[i].action = visualization_msgs::Marker::ADD;
		marker.markers[i].pose.position.x = pos_wall[i].x;
		marker.markers[i].pose.position.y = pos_wall[i].y;
		marker.markers[i].pose.position.z = wall_high/2;
		marker.markers[i].pose.orientation.x = marker.markers[i].pose.orientation.y = marker.markers[i].pose.orientation.z = 0.0;
		marker.markers[i].pose.orientation.w = 1.0;
		marker.markers[i].scale.x = 0.01;
		marker.markers[i].scale.y = 0.01;
		marker.markers[i].scale.z = wall_high;
		marker.markers[i].color.a = alpha;
		marker.markers[i].color.r = rgb_value.x;
		marker.markers[i].color.g = rgb_value.y;
		marker.markers[i].color.b = rgb_value.z;
		
		
		//target_posのtfを定義し、送信する
		
		
		free_base_tf[i].frame_id_ = "laser";
		free_base_tf[i].child_frame_id_ = "free_space" + value2string(i);
		free_base_tf[i].stamp_ = ros::Time::now();
		free_base_tf[i].setRotation(tf::createQuaternionFromRPY(0,0,((3 * PI / 2) / 750) * i - (PI / 4) - (PI / 2) + (PI / 4)));
		tf_pub_.sendTransform(free_base_tf[i]);
		
		if(isinf(sensor_msg.ranges[i]))
		{
			alpha = 0.5;
			free_space_high[i] = 5;
		}
		else
		{
			
			free_space_high[i] = sensor_msg.ranges[i];
			alpha = 0.5;
			
			
		}
		
		
		marker.markers[i+512].header.frame_id = "free_space" + value2string(i);
		marker.markers[i+512].header.stamp = ros::Time();
		marker.markers[i+512].ns = "free_space";
		marker.markers[i+512].id = i+512;
		marker.markers[i+512].type = visualization_msgs::Marker::CUBE;
		marker.markers[i+512].action = visualization_msgs::Marker::ADD;
		marker.markers[i+512].pose.position.x = free_space_high[i]/2;
		marker.markers[i+512].pose.position.y = 0;
		marker.markers[i+512].pose.position.z = 0;
		marker.markers[i+512].pose.orientation.x = marker.markers[i+512].pose.orientation.y = marker.markers[i+512].pose.orientation.z = 0.0;
		marker.markers[i+512].pose.orientation.w = 1.0;
		marker.markers[i+512].scale.x = free_space_high[i];
		marker.markers[i+512].scale.y = 0.01;
		marker.markers[i+512].scale.z = 0.01;
		marker.markers[i+512].color.a = alpha;
		marker.markers[i+512].color.r = 0.8;
		marker.markers[i+512].color.g = 0.8;
		marker.markers[i+512].color.b = 0.8;
		
		
		//marker.markers[0].text = test;
    
    
	}
	
	marker_pub.publish( marker );
	
	
	//ROS_INFO("cos:%f,sin:%f",cos(((3 * PI / 2) / 511) * 256),sin(((3 * PI / 2) / 511) * 256));
}



lrf_description::lrf_description() {
    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/urg_description/markers", 0 );
    
    scan_sub = n.subscribe("/scan", 1, &lrf_description::scan_callback,this);
}
lrf_description::~lrf_description() {

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "lrf_description");
    ros::NodeHandle n;

    lrf_description lrf_description_class;

    ros::spin();

    return 0;
};

