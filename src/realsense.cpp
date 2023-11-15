#include "realsense.h"


Depth2PC::Depth2PC() : Node("depth_to_pc_realsense")
{
	// Инициализация параметров
	this->declare_parameter("depth_topic", "/camera/depth/image_rect_raw");
	depth_topicname = this->get_parameter("depth_topic").as_string();

	this->declare_parameter("camera_info", "/camera/depth/camera_info");
	camerainfo_topicname = this->get_parameter("camera_info").as_string();

	this->declare_parameter("pc_topic", "/depth2pc/pc");
	publisher_topicname = this->get_parameter("pc_topic").as_string();

	this->declare_parameter("output_frame_id", "camera_link");
	output_frame_id = this->get_parameter("output_frame_id").as_string();


	subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
				depth_topicname, 2, std::bind(&Depth2PC::depth_callback, this, _1));
	subscriber_ci = this->create_subscription<sensor_msgs::msg::CameraInfo>(
				camerainfo_topicname,2, std::bind(&Depth2PC::info_callback,this, _1));
	publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publisher_topicname, 10);

	RCLCPP_INFO(this->get_logger(), "Node started");
}

Depth2PC::~Depth2PC(){}

void Depth2PC::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	rclcpp::Time begin = this->get_clock()->now();

	if ( camera_info == nullptr )
	{
		RCLCPP_WARN(this->get_logger(), "Camera's info is null, skipping pointcloud concersation");
		return;
	}
	img = msg;

	sensor_msgs::msg::PointCloud2 cloud_msg;
	//pc = std::shared_ptr<sensor_msgs::msg::PointCloud2>{&cloud_msg};

	convert<uint16_t>(img, &cloud_msg, camera_info);

	publisher_->publish(cloud_msg);

	rclcpp::Time end = this->get_clock()->now();
	RCLCPP_INFO(this->get_logger(), "Время вычислений: %f", end.seconds()-begin.seconds());
}

void Depth2PC::info_callback(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
	camera_info = info;
}

template<typename T>
void Depth2PC::convert(sensor_msgs::msg::Image::SharedPtr image,
						sensor_msgs::msg::PointCloud2 *pc,
						sensor_msgs::msg::CameraInfo::SharedPtr info)
{
	// init pointcloud
	pc->header = image->header;
	pc->header.frame_id = output_frame_id;
    //RCLCPP_INFO(this->get_logger(), "Image size is %dx%d px", image->width, image->height);
	pc->height = image->height;
	pc->width = image->width;
	pc->is_dense = false;
	pc->is_bigendian = false;
	pc->fields.clear();
	pc->fields.reserve(1);

	double fx = info->k[0],
			fy = info->k[4],
			cx = info->k[2],
			cy = info->k[5];

	// Setup fields
	sensor_msgs::PointCloud2Modifier modifier(*pc);
	modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
	modifier.resize(pc->height*pc->width);
    pc->height = image->height;
	pc->width = image->width;
	pc->row_step = pc->width*32;

	//iterators
	sensor_msgs::PointCloud2Iterator<float> out_x(*pc, "x");
	sensor_msgs::PointCloud2Iterator<float> out_y(*pc, "y");
	sensor_msgs::PointCloud2Iterator<float> out_z(*pc, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*pc, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*pc, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*pc, "b");

	

	const T * depth_row = reinterpret_cast<const T *>(&image->data[0]);
	int row_step = image->step / sizeof(T);

	// image->encoding
	for (size_t m = 0; m < image->height; m++, depth_row += row_step)
	{
		for (size_t n = 0; n < img->width; n++)
		{
		T depth = depth_row[n];
		
		float x,y,z;

		if (std::isfinite(depth) && depth != 0 )
		{
			float d = float(depth)*0.001;
			
			y = -(n-cx)*d/fx;
			z = -(m-cy)*d/fy;
			x = d;
			
		}
		else
		{
			x=y=z = std::numeric_limits<float>::quiet_NaN();
		}
		*out_x = x;
		*out_y = y;
		*out_z = z;

		*out_r = 255;
		*out_g = 255;
		*out_b = 255;

		++out_x;
		++out_y;
		++out_z;
		++out_r;
		++out_g;
		++out_b;
		}
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Depth2PC>());
	rclcpp::shutdown();
	return 0;
}
