# yolo_node.py 또는 lane_detection_node.py 파일

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):

    def __init__(self):
        super().__init__("lane_detection_node")
        
        self.cv_bridge = CvBridge()

        self.create_subscription(
            Image,
            "image_raw",
            self.image_cb,
            10
        )

        # --- 변경점 1: 디버깅용 Publisher 추가 ---
        self._lane_pub = self.create_publisher(Image, "image_lane", 10)
        self._roi_pub = self.create_publisher(Image, "image_roi", 10)
        self._canny_pub = self.create_publisher(Image, "image_canny", 10)
        # -----------------------------------------

        self.get_logger().info('LaneDetectionNode has been started with debugging publishers.')

    def image_cb(self, msg: Image) -> None:
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        # --- 변경점 2: 중간 이미지를 함께 반환받음 ---
        lane_image, roi_image, canny_image = self.process_image_for_lane_detection(cv_image)
        # ---------------------------------------------

        try:
            # 최종 차선 인식 이미지 발행
            lane_msg = self.cv_bridge.cv2_to_imgmsg(lane_image, "bgr8")
            lane_msg.header = msg.header
            self._lane_pub.publish(lane_msg)

            # --- 디버깅용 이미지들을 각각 발행 ---
            # ROI 이미지는 단일 채널이므로 'mono8'로 인코딩하여 발행
            roi_msg = self.cv_bridge.cv2_to_imgmsg(roi_image, "mono8")
            roi_msg.header = msg.header
            self._roi_pub.publish(roi_msg)

            # Canny 엣지 이미지도 'mono8'로 인코딩하여 발행
            canny_msg = self.cv_bridge.cv2_to_imgmsg(canny_image, "mono8")
            canny_msg.header = msg.header
            self._canny_pub.publish(canny_msg)
            # ------------------------------------

        except Exception as e:
            self.get_logger().error(f"Failed to publish images: {e}")

    def process_image_for_lane_detection(self, image):
            height, width = image.shape[:2]
            
            # --- 1. 색상 필터링으로 변경 (기존 그레이스케일 코드 대체) ---
            # BGR 이미지를 HSV 색상 공간으로 변환
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # << 중요 >>
            # 아래 값들은 시작점일 뿐이며, 제공해드린 calibration_tuner.py로
            # 직접 찾은 최적의 값을 여기에 적용해야 합니다.
            lower_yellow = np.array([0, 59, 106])
            upper_yellow = np.array([96, 200, 255])

            lower_white = np.array([0, 0, 180])
            upper_white = np.array([255, 30, 255])

            # 노란색 마스크와 흰색 마스크를 각각 생성
            yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
            
            # 두 마스크를 하나로 합침
            color_mask = cv2.bitwise_or(yellow_mask, white_mask)
            # ----------------------------------------------------

            # 블러 처리는 기존 gray_image 대신 color_mask에 적용
            blur_image = cv2.GaussianBlur(color_mask, (5, 5), 0)
            
            # 캐니 엣지 검출은 블러 처리된 마스크 이미지에 적용
            canny_image = cv2.Canny(blur_image, 30, 90)
            
            # --- 이후 로직은 기존과 동일 ---
            roi_vertices = np.array([[(0, height), (width / 2, height / 2 + 50), (width, height)]], dtype=np.int32)
            masked_image = self.region_of_interest(canny_image, roi_vertices)
            
            lines = cv2.HoughLinesP(masked_image, 1, np.pi / 180, 50, np.array([]), minLineLength=40, maxLineGap=100)
            
            line_image = np.zeros_like(image)
            if lines is not None:
                line_image = self.draw_lines_on_image(line_image, lines, image.shape[0])

            combined_image = cv2.addWeighted(image, 0.8, line_image, 1, 1)

            # 디버깅을 위해 기존 masked_image 대신 color_mask를 반환하면 더 유용합니다.
            return combined_image, color_mask, canny_image
        
    def region_of_interest(self, image, vertices):
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, vertices, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    # draw_lines 함수를 2개로 분리하여 재사용성 높임
    def draw_lines_on_image(self, image, lines, height):
        left_fit = []
        right_fit = []
        
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < -0.5:
                left_fit.append((slope, intercept))
            elif slope > 0.5:
                right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0) if left_fit else None
        right_fit_average = np.average(right_fit, axis=0) if right_fit else None

        if left_fit_average is not None:
            left_line = self.make_coordinates(height, left_fit_average)
            cv2.line(image, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 0, 255), 10)
        
        if right_fit_average is not None:
            right_line = self.make_coordinates(height, right_fit_average)
            cv2.line(image, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 0, 255), 10)
        return image
        
    def make_coordinates(self, height, line_parameters):
        slope, intercept = line_parameters
        y1 = height
        y2 = int(y1 * (3/5))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return np.array([x1, y1, x2, y2])


def main():
    rclpy.init()
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()