#include "kf.hpp"
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <estimator/ParabolicTrackerAction.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <vicon_bridge/Marker.h>
#include <vicon_bridge/Markers.h>
namespace et {
class Estimator {
private:
protected:
  // Node handle
  ros::NodeHandle nh_;
  /// @brief Action server handle
  actionlib::SimpleActionServer<estimator::ParabolicTrackerAction>
      action_server_;
  /// @brief Name of the action
  std::string name_;

  estimator::ParabolicTrackerFeedback feedback_;
  std::string objectName_;
  estimator::ParabolicTrackerResult result_;

  /// @brief Subscriber for the vicon msgs
  ros::Subscriber vicon_sub_;
  /// @brief Timer meant to handle timeouts
  ros::Timer action_timer_;
  ros::Timer flight_timer_;

  linearKF::KF filter_;
  /// @brief Store the previous msgs
  std::vector<std::pair<geometry_msgs::Point, double>> msg_hist_;
  /// @brief Store the filtered history
  std::vector<geometry_msgs::Point> filtered_hist_;
  /// @brief Total flight time elapsed
  double flight_time_;

  /// @brief Populate the feedback_
  /// @param isValid
  /// @param deltaAltitude
  /// @param interceptDelta
  /// @param interceptTime
  /// @param target
  /// @param intercept
  void populateFeedback(const bool &isValid, const std::string name,
                        const double &deltaAltitude,
                        const double &interceptDelta,
                        const double &interceptTime,
                        const geometry_msgs::Point &currentPosition,
                        const std::vector<geometry_msgs::Point> &predictedTrajectory) {
    this->feedback_.isValid = isValid;
    this->feedback_.objectName = name;
    this->feedback_.deltaAltitude = deltaAltitude;
    this->feedback_.interceptDelta = interceptDelta;
    this->feedback_.interceptTime = interceptTime;
    this->feedback_.currentPosition = currentPosition;
    this->feedback_.predictedTrajectory = predictedTrajectory;
  }
  void clearFeedback() {
    this->populateFeedback(false, "", 0.0, 0.0, 0.0,
                           geometry_msgs::Point(),
                           std::vector<geometry_msgs::Point>());
  }
  void simulateFlight_(std::vector<geometry_msgs::Point> *prediction);
  void eigenToPoint(const Eigen::VectorXd &e, geometry_msgs::Point *p){
    p->x = e(0);
    p->y = e(1);
    p->z = e(2);
  }
  void resetAccumulators_(){
    this->msg_hist_.clear();
    this->filtered_hist_.clear();
    this->flight_time_ = 0.0;
    this->filter_ = linearKF::KF();
  }
  double startPredictionTime_;
  double startPredictionAltitude_;
public:
  /// @brief
  /// @param name
  Estimator(std::string name);
  ///
  ~Estimator();
  /// @brief A timeout callback meant to reset the action server
  void actiontimerCallback(const ros::TimerEvent &);
  /// @brief A timeout callback meant to reset throws
  void flighttimerCallback(const ros::TimerEvent &);
  /// @brief Callback when a goal is received from a client
  void goalCallback();
  void preemptCallback();
  /// @brief Callback when a markers msg is received from the vicon system
  /// @param markers
  void markersCallback(const vicon_bridge::MarkersConstPtr &markers);
};
} // namespace et
