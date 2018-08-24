#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

#include <boost/thread/thread.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "gidisizi/utils/Node.hpp"
#include "gidisizi/utils/Graph.hpp"
#include "gidisizi/utils/Curve.hpp"


namespace path_vis{
  inline void drawCurve( cv::Mat img,gidisizi::Curve* c, cv::Scalar color
                , int division, int w = 800, int thickness = 2, int lineType = 8){
    std::vector<Eigen::VectorXd> points =  c->getDrawable(division);
    for (int i = 0; i<points.size()-1;i++){
      auto p1 = points[i];
      auto p2 = points[i+1];
      cv::line(img,
        cv::Point(w / 2 * (1 + p1[0]), w / 2 * (1 - p1[1])),
        cv::Point(w / 2 * (1 + p2[0]), w / 2 * (1 - p2[1])),
        color, thickness, lineType);
    }
  }

  template<typename Planner,typename Environment>
  inline void drawGraph( Planner& planner, Environment* environment)
  {
    int w = 800;
    int thickness = 2;
    int lineType = 8;
    cv::Scalar graphColor = cv::Scalar(100, 100, 100);
    cv::Scalar solution = cv::Scalar(255, 0, 0);
    cv::Scalar goalcolor = cv::Scalar(255, 0, 255);
    cv::Scalar wallcolor = cv::Scalar(0, 0, 255);

    cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);

    for (auto wall : environment->getWalls()) {
      cv::rectangle(img, cv::Point(w / 2 * (1 + (wall.point1[0])), w / 2 * (1 - (wall.point1[1]))),
                    cv::Point(w / 2 * (1 + (wall.point2[0])), w / 2 * (1 - (wall.point2[1]))), wallcolor);
    }
    // Goal
    cv::rectangle(
        img,
        cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] - planner.getSolutionAccuracy()[0])),
                  w / 2 * (1 - (planner.getGoal()->getState()[1] - planner.getSolutionAccuracy()[1]))),
        cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] + planner.getSolutionAccuracy()[0])),
                  w / 2 * (1 - (planner.getGoal()->getState()[1] + planner.getSolutionAccuracy()[1]))),
        goalcolor);
    cv::imshow("Display window", img);
    cv::waitKey(1);
    for (auto e : planner.getGraph().getEdges()) {
      cv::line(img,
               cv::Point(w / 2 * (1 + e.start->getState()[0]), w / 2 * (1 - e.start->getState()[1])),
               cv::Point(w / 2 * (1 + e.end->getState()[0]), w / 2 * (1 - e.end->getState()[1])),
               graphColor, thickness, lineType);
      cv::imshow("Display window", img);
      cv::waitKey(1);
    }
    const auto path = planner.getPath();
    for (int i = 0; i < path.size() - 1; i++) {
      auto start = path[i];
      auto end = path[i + 1];
      cv::line(img, cv::Point(w / 2 * (1 + start->getState()[0]), w / 2 * (1 - start->getState()[1])),
               cv::Point(w / 2 * (1 + end->getState()[0]), w / 2 * (1 - end->getState()[1])),
               solution, thickness * 2, lineType);
      cv::imshow("Display window", img);
      cv::waitKey(1);
    }

    cv::waitKey(0);
  }



  /*
  template<typename Planner,typename Environment>
  inline void drawGraphEvolution(Planner& planner,Environment* environment)
  {
    int w = 800;
    int thickness = 2;
    int lineType = 8;
    cv::Scalar graphColor = cv::Scalar(100, 100, 100);
    cv::Scalar solution = cv::Scalar(255, 0, 0);
    cv::Scalar goalcolor = cv::Scalar(255, 0, 255);
    cv::Scalar wallcolor = cv::Scalar(0, 0, 255);
    cv::Scalar  carColor = cv::Scalar(0, 255, 255);
    cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);

    for (auto wall : environment->getWalls()) {
      cv::rectangle(img, cv::Point(w / 2 * (1 + (wall.point1[0])), w / 2 * (1 - (wall.point1[1]))),
                    cv::Point(w / 2 * (1 + (wall.point2[0])), w / 2 * (1 - (wall.point2[1]))),
                    wallcolor);
    }
    // Goal
    cv::rectangle(
        img,
        cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] - planner.getSolutionAccuracy()[0])),
                  w / 2 * (1 - (planner.getGoal()->getState()[1] - planner.getSolutionAccuracy()[1]))),
        cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] + planner.getSolutionAccuracy()[0])),
                  w / 2 * (1 - (planner.getGoal()->getState()[1] + planner.getSolutionAccuracy()[1]))),
        goalcolor);
    cv::imshow("Display window", img);
    cv::waitKey(1);
    int index = 0 ;
    const auto debugPaths = planner.getDebugPaths();
    for(auto G : planner.getDebugGraphes()){

      img = cv::Mat::zeros( img.size(), img.type() );
      for (auto wall : environment->getWalls()) {
        cv::rectangle(img, cv::Point(w / 2 * (1 + (wall.point1[0])), w / 2 * (1 - (wall.point1[1]))),
                      cv::Point(w / 2 * (1 + (wall.point2[0])), w / 2 * (1 - (wall.point2[1]))),
                      wallcolor);
      }
      // Goal
      cv::rectangle(
          img,
          cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] - planner.getSolutionAccuracy()[0])),
                    w / 2 * (1 - (planner.getGoal()->getState()[1] - planner.getSolutionAccuracy()[1]))),
          cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] + planner.getSolutionAccuracy()[0])),
                    w / 2 * (1 - (planner.getGoal()->getState()[1] + planner.getSolutionAccuracy()[1]))),
          goalcolor);
      for (auto e : G.getEdges()) {
        cv::line(img,
                 cv::Point(w / 2 * (1 + e.start->getState()[0]), w / 2 * (1 - e.start->getState()[1])),
                 cv::Point(w / 2 * (1 + e.end->getState()[0]), w / 2 * (1 - e.end->getState()[1])),
                 graphColor, thickness, lineType);

      }

      for (int i = 0; i < debugPaths[index].size() - 1; i++) {
        auto start = debugPaths[index][i];
        auto end = debugPaths[index][i + 1];
        cv::line(img, cv::Point(w / 2 * (1 + start->getState()[0]), w / 2 * (1 - start->getState()[1])),
                 cv::Point(w / 2 * (1 + end->getState()[0]), w / 2 * (1 - end->getState()[1])),
                 solution, thickness * 5, lineType);

        if (start->getState().size()==3){
          double x = cos(start->getState()[2])*0.2;
          double y = sin(start->getState()[2])*0.2;
          cv::arrowedLine(img, cv::Point( w / 2 * (1 + start->getState()[0]-x)
                                        , w / 2 * (1 - start->getState()[1]-y))
                             , cv::Point( w / 2 * (1 + start->getState()[0]+x)
                                        , w / 2 * (1 - start->getState()[1]+y))
                             , carColor, thickness * 5);
        }

      }
      cv::imshow("Display window", img);
      cv::waitKey(1);
      index++;//std::cerr<<(index++)<<std::endl;
    }
    std::cerr<<"Press a key to stop!!"<<std::endl;
    cv::waitKey(0);
  }
  */

  template<typename Planner,typename Environment>
  inline void drawGraphEvolution(Planner& planner,Environment* environment)
  {
    int w = 800;
    int thickness = 2;
    int lineType = 8;
    cv::Scalar graphColor = cv::Scalar(100, 100, 100);
    cv::Scalar solution = cv::Scalar(255, 0, 0);
    cv::Scalar goalcolor = cv::Scalar(255, 0, 255);
    cv::Scalar wallcolor = cv::Scalar(0, 0, 255);
    cv::Scalar  carColor = cv::Scalar(0, 255, 255);
    cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);

    for (auto wall : environment->getWalls()) {
      cv::rectangle(img, cv::Point(w / 2 * (1 + (wall.point1[0])), w / 2 * (1 - (wall.point1[1]))),
                    cv::Point(w / 2 * (1 + (wall.point2[0])), w / 2 * (1 - (wall.point2[1]))),
                    wallcolor);
    }
    // Goal
    cv::rectangle(
        img,
        cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] - planner.getSolutionAccuracy()[0])),
                  w / 2 * (1 - (planner.getGoal()->getState()[1] - planner.getSolutionAccuracy()[1]))),
        cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] + planner.getSolutionAccuracy()[0])),
                  w / 2 * (1 - (planner.getGoal()->getState()[1] + planner.getSolutionAccuracy()[1]))),
        goalcolor);
    cv::imshow("Display window", img);
    cv::waitKey(1);
    int index = 0 ;
    const auto debugPaths = planner.getDebugPaths();
    for(auto G : planner.getDebugGraphes()){
      img = cv::Mat::zeros( img.size(), img.type() );
      for (auto wall : environment->getWalls()) {
        cv::rectangle(img, cv::Point(w / 2 * (1 + (wall.point1[0])), w / 2 * (1 - (wall.point1[1]))),
                      cv::Point(w / 2 * (1 + (wall.point2[0])), w / 2 * (1 - (wall.point2[1]))),
                      wallcolor);
      }
      // Goal
      cv::rectangle(
          img,
          cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] - planner.getSolutionAccuracy()[0])),
                    w / 2 * (1 - (planner.getGoal()->getState()[1] - planner.getSolutionAccuracy()[1]))),
          cv::Point(w / 2 * (1 + (planner.getGoal()->getState()[0] + planner.getSolutionAccuracy()[0])),
                    w / 2 * (1 - (planner.getGoal()->getState()[1] + planner.getSolutionAccuracy()[1]))),
          goalcolor);
      for (auto e : G.getEdges()) {
        drawCurve(img,e,graphColor,10);
      }
      for (int i = 1; i < debugPaths[index].size() ; i++) {
        auto end = debugPaths[index][i];
        drawCurve(img,end->getPathToThis(),solution,10);

      }
      cv::imshow("Display window", img);
      cv::waitKey(1);
      index++;//std::cerr<<(index++)<<std::endl;
    }
    std::cerr<<"Press a key to stop!!"<<std::endl;
    cv::waitKey(0);
  }



  inline void drawTest( std::vector<gidisizi::Curve*> curves)
  {
    int w = 800;
    int thickness = 2;
    int lineType = 8;
    cv::Scalar graphColor = cv::Scalar(100, 100, 100);
    cv::Scalar solution = cv::Scalar(255, 0, 0);
    cv::Scalar goalcolor = cv::Scalar(255, 0, 255);
    cv::Scalar wallcolor = cv::Scalar(0, 0, 255);

    cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);
    for (auto c : curves) {
      drawCurve(img,c,graphColor,20);
    }
    cv::imshow("Curve Test", img);

    cv::waitKey(0);
  }



}// namespace path_vis
