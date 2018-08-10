#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_front_, image_pub_short_, image_pub_long_;
	ros::Subscriber compressed_sub_, long_focus_sub_;

public:
	ImageConverter()
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/simulator/camera_node/image2", 1, &ImageConverter::ImageCallback, this);
		image_pub_front_ = it_.advertise("/apollo/sensor/camera/obstacle/front_6mm", 1);
		image_pub_short_ = it_.advertise("/apollo/sensor/camera/traffic/image_short", 1);
		image_pub_long_ = it_.advertise("/apollo/sensor/camera/traffic/image_long", 1);				
		compressed_sub_ = nh_.subscribe("/simulator/camera_node/image/compressed", 1, &ImageConverter::CompressedImageCallback, this);
		long_focus_sub_ = nh_.subscribe("/simulator/camera_node/long_focus/compressed", 1, &ImageConverter::LongFocusCallback, this);
	}

	~ImageConverter(){}

	static unsigned int clamp(float num) {
		if (num > 255) {
			return 255;
		}
		else if (num < 0) {
			return 0;
		}
		return (unsigned int)num;
	}

	void CompressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
		cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);

		sensor_msgs::Image raw_yuyv_msg;
		raw_yuyv_msg = ConvertRgbToYuyv(image);
		raw_yuyv_msg.header = msg->header;
		
		image_pub_front_.publish(raw_yuyv_msg);
		image_pub_short_.publish(raw_yuyv_msg);
	}

	void LongFocusCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
		cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);

		sensor_msgs::Image raw_yuyv_msg;
		raw_yuyv_msg = ConvertRgbToYuyv(image);
		raw_yuyv_msg.header = msg->header;

		image_pub_long_.publish(raw_yuyv_msg);
	}

	sensor_msgs::Image ConvertRgbToYuyv(const cv::Mat image) {
		sensor_msgs::Image raw_yuyv_msg;
		
		raw_yuyv_msg.height = image.rows;
		raw_yuyv_msg.width = image.cols;
		raw_yuyv_msg.step = image.cols*image.channels();
		raw_yuyv_msg.encoding = "yuyv";

		float y0, u0, v0, y1, u1, v1, u, v;
		unsigned int r0, g0, b0, r1, g1, b1;

		for (unsigned int y = 0; y < image.rows; y++){
			for (unsigned int x = 0; x < image.cols; x+=2){
				cv::Vec3b px0 = image.at<cv::Vec3b>(y,x);
				cv::Vec3b px1 = image.at<cv::Vec3b>(y,x+1);

				b0 = px0[0];
				g0 = px0[1];
				r0 = px0[2];
				b1 = px1[0];
				g1 = px1[1];
				r1 = px1[2];

				y0 = 0.303579*r0 + 0.595598*g0 + 0.100823*b0;
				y1 = 0.303579*r1 + 0.595598*g1 + 0.100823*b1;
				u0 = -0.14874*r0 - 0.291817*g0 + 0.440557*b0;
				u1 = -0.14874*r1 - 0.291817*g1 + 0.440557*b1;
				v0 = 0.495144*r0 - 0.423461*g0 - 0.0716834*b0;
				v1 = 0.495144*r1 - 0.423461*g1 - 0.0716834*b1;
				u = (u0 + u1)/2 + 128;
				v = (v0 + v1)/2 + 128;

				// swap to y u y v if avx2
				raw_yuyv_msg.data.push_back(clamp(y0));
				raw_yuyv_msg.data.push_back(clamp(u));
				raw_yuyv_msg.data.push_back(clamp(y1));
				raw_yuyv_msg.data.push_back(clamp(v));

			}
		}

		return raw_yuyv_msg;
	}

	void ImageCallback(const sensor_msgs::ImageConstPtr &msg){

		sensor_msgs::Image yuyv_msg;
		yuyv_msg.header = msg->header;
		yuyv_msg.height = msg->height;
		yuyv_msg.width = msg->width;
		yuyv_msg.step = msg->step;
		yuyv_msg.encoding = "yuyv";

		float y0, u0, v0, y1, u1, v1, u, v;
		unsigned int r0, g0, b0, r1, g1, b1;

		for (unsigned int y = 0; y < msg->height; y++) {
			const unsigned char* row = &msg->data[y * msg->step];
				for (unsigned int x = 0; x < msg->width; x+=2) {
					b0 = row[3*x+0]; 
					g0 = row[3*x+1];
					r0 = row[3*x+2];
					b1 = row[3*x+3];
					g1 = row[3*x+4];
					r1 = row[3*x+5];

					y0 = 0.303579*r0 + 0.595598*g0 + 0.100823*b0;
					y1 = 0.303579*r1 + 0.595598*g1 + 0.100823*b1;
					u0 = -0.14874*r0 - 0.291817*g0 + 0.440557*b0;
					u1 = -0.14874*r1 - 0.291817*g1 + 0.440557*b1;
					v0 = 0.495144*r0 - 0.423461*g0 - 0.0716834*b0;
					v1 = 0.495144*r1 - 0.423461*g1 - 0.0716834*b1;
					u = (u0 + u1)/2 + 128;
					v = (v0 + v1)/2 + 128;

					yuyv_msg.data.push_back(clamp(y0));
					yuyv_msg.data.push_back(clamp(u));			
					yuyv_msg.data.push_back(clamp(y1));
					yuyv_msg.data.push_back(clamp(v));
			}
		} 

		image_pub_front_.publish(yuyv_msg);
		image_pub_short_.publish(yuyv_msg);
		image_pub_long_.publish(yuyv_msg);

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
