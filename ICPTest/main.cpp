/**
 * このプログラムは、以下のサイトに掲載されていたもの
 * http://book.mynavi.jp/support/pc/opencv2/c3/opencv_linalg.html
 */

#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {
  cv::Size img_size(500, 500);
  cv::Mat img = cv::Mat::zeros(img_size, CV_8UC3);

  cv::Mat_<float> points(2, 3);
  points(0, 0) = 100.0f;
  points(0, 1) = 100.0f;
  points(1, 0) = 200.0f;
  points(1, 1) = 100.0f;
  /*
  points(2, 0) = 200.0f;
  points(2, 1) = 200.0f;
  points(3, 0) = 100.0f;
  points(3, 1) = 200.0f;
  */
  
  for(int i=0; i<points.rows; ++i) {
	points(i, 2) = 1.0f;

    // 変換前の座標点を描画（水色の点）
    cv::circle(img, cv::Point(points(i,0), points(i,1)), 2, cv::Scalar(200,200,0), -1, CV_AA);
  }

  cv::Mat_<float> dst_points(2, 2);
  dst_points(0, 0) = 150.0f;
  dst_points(0, 1) = 150.0f;
  dst_points(1, 0) = 250.0f;
  dst_points(1, 1) = 250.0f;
  /*
  dst_points(2, 0) = 150.0f;
  dst_points(2, 1) = 350.0f;
  dst_points(3, 0) = 50.0f;
  dst_points(3, 1) = 250.0f;
  */

  // 変換後の座標点を描画（赤色の点）
  for(int i=0; i<dst_points.rows; ++i) {
    cv::circle(img, cv::Point(dst_points(i,0), dst_points(i,1)), 2, cv::Scalar(50,50,255), -1, CV_AA);
  }

  cv::Mat_<float> src_points(points, cv::Rect(0,0,2,points.rows));
  std::cout << src_points << std::endl;

  // 2次元アフィン変換の推定（回転，並進，拡大縮小の拘束あり）
  cv::Mat est_matrix  = cv::estimateRigidTransform(src_points.reshape(2), dst_points.reshape(2), false);
  
  std::cout << "Estimated Matrix:" << std::endl;
  std::cout << est_matrix << std::endl << std::endl;

  /// 元の点を推定した変換行列で変換する
  cv::Mat est_matrixF, est_matrixF_full;
  est_matrix.convertTo(est_matrixF, CV_32F);

  cv::Mat_<float> est_points = points * est_matrixF.t();

  for(int i=0; i<est_points.rows; ++i) {
    // 拘束ありのアフィン変換で変換した点集合を描画（黄色の円）
    cv::circle(img, cv::Point(est_points(i,0), est_points(i,1)), 5, cv::Scalar(50,255,255), 1, CV_AA);
  }

  cv::namedWindow("image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::imshow("image", img);
  cv::waitKey(0);
}