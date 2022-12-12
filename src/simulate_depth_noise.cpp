
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#define VISUALIZE_OUTPUT

image_transport::Publisher pub;

void addSinWave(cv::Mat &mat, float mag, float freq_r, float freq_c, float phase)
{
  //freq_r *= 2*M_PI;
  //freq_c *= 2*M_PI;
  mat.forEach<float>([mat, mag, freq_r, freq_c, phase](float& pixel, const int position[]) -> void {
    float row = float(position[0]); // / mat.rows;
    float col = float(position[1]); // / mat.cols;
    pixel = pixel + mag*sin(freq_r*row + freq_c*col + phase);
  });
}

void mulSinWave(cv::Mat &mat, float mag, float freq_r, float freq_c, float phase)
{
  //freq_r *= 2*M_PI;
  //freq_c *= 2*M_PI;
  mat.forEach<float>([mat, mag, freq_r, freq_c, phase](float& pixel, const int position[]) -> void {
    float row = float(position[0]); // / mat.rows;
    float col = float(position[1]); // / mat.cols;
    pixel = pixel * mag*sin(freq_r*row + freq_c*col + phase);
  });
}

void filterMat1(cv::Mat &mat)
{
  mat.forEach<float>([](float& pixel, const int position[]) -> void {
    if (std::isnan(pixel) || std::isinf(pixel) || pixel==0.0)
    {
      pixel = 1000.0;
    }
  });
}

void filterMat2(cv::Mat &mat)
{
  mat.forEach<float>([](float& pixel, const int position[]) -> void {
    if (pixel>5.0)
    {
      pixel = std::numeric_limits<float>::quiet_NaN();
      //pixel = 0.0;
    }
  });
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // check msg->encoding.c_str()
  uint8_t* data_copy = new uint8_t[msg->data.size()];
  std::memcpy(data_copy, msg->data.data(), msg->data.size());
  cv::Mat mat(msg->height, msg->width, CV_32F, data_copy);

  // filter nans, infs, zeros
  filterMat1(mat);

  cv::Mat addition_noise = cv::Mat::zeros(msg->height, msg->width, CV_32F);
  cv::Mat multiply_noise1 = cv::Mat(msg->height, msg->width, CV_32F);
  cv::Mat multiply_noise2 = cv::Mat(msg->height, msg->width, CV_32F);
  mat.copyTo(multiply_noise1);
  mat.copyTo(multiply_noise2);

  // additive noise
  static float phase1 = 0.0; phase1 = std::fmod(phase1+1.1, M_PI*2);
  addSinWave(addition_noise, 0.002, 5.3, 5.2, phase1);

  static float phase2 = 0.0; phase2 += std::fmod(phase2+2.3, M_PI*2);
  addSinWave(addition_noise, 0.0015, 0.07, 0.1, phase2);

  static float phase3 = 0.0; phase3 += std::fmod(phase3+2.9, M_PI*2);
  addSinWave(addition_noise, 0.001, 0.1, 0.5, phase3);

  // multiplicative noise
  static float phase4 = 0.0; phase4 += std::fmod(phase4+1.9, M_PI*2);
  mulSinWave(multiply_noise1, 0.002, 0.2, 3.3, phase4);

  static float phase5 = 0.0; phase5 += std::fmod(phase5+1.7, M_PI*2);
  mulSinWave(multiply_noise2, 0.007, 0.22, 0.1, phase5);

  // add noises
  mat = mat + addition_noise + multiply_noise1 + multiply_noise2;

  // add shadowing effect
  cv::GaussianBlur(mat, mat, cv::Size(3, 3), 0);

  // add gaussian noise
  cv::Mat addition_gaussian = cv::Mat::zeros(msg->height, msg->width, CV_32F);
  cv::Mat multiply_gaussian = cv::Mat::zeros(msg->height, msg->width, CV_32F);
  cv::randn(addition_gaussian, cv::Scalar::all(0.0), cv::Scalar::all(0.001));
  cv::randn(multiply_gaussian, cv::Scalar::all(0.0), cv::Scalar::all(0.003));
  mat = mat + mat.mul(multiply_gaussian) + addition_gaussian;

  // limit values
  filterMat2(mat);


#ifdef VISUALIZE_OUTPUT
  cv::imshow("view", mat);
  cv::waitKey(1);
#endif

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "32FC1", mat).toImageMsg();
  pub.publish(out_msg);

  delete data_copy;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulate_depth_noise_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

#ifdef VISUALIZE_OUTPUT
  cv::namedWindow("view");
#endif

  image_transport::ImageTransport it(nh);
  pub = it.advertise("/camera/depth/image_raw_noisy", 1);
  image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
  ros::spin();

#ifdef VISUALIZE_OUTPUT
  cv::destroyWindow("view");
#endif

  return 0;
}