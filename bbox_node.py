import matplotlib.pyplot as plt
import torch
import cv2
from copy import deepcopy
from sensor_msgs.msg import Image
import rclpy

from rclpy.node import Node


# from interfaces.msg import LivoxPointMsg, LidarBboxMsg, BboxMsg


plot_path = "/home/inspirationagx01/Robot-X-YoloV5/perception/plots"

test_image_path = "/home/inspirationagx01/Robot-X-YoloV5/data/test/images/9_jpg.rf.0742fd2b9691e8f00d5814ebac2fb097.jpg"
test_label_path = "/home/inspirationagx01/Robot-X-YoloV5/data/test/labels/9_jpg.rf.0742fd2b9691e8f00d5814ebac2fb097.txt"


class Detection(Node):
    # TODO: allow more classes
    label2class = {
        '0': 'blue-buoy',
        '1': 'pink-buoy',
        '2': 'white-buoy'
    }

    label2color = {
        '0': (0, 0, 255),
        '1': (255, 0, 0),
        '2': (0, 255, 0)
    }

    intrinsic_path = "/home/inspirationagx01/Robot-X-YoloV5/calibration/intrinsic.txt"
    extrinsic_path = "/home/inspirationagx01/Robot-X-YoloV5/calibration/extrinsic.txt"

    weights_path = '/home/inspirationagx01/Robot-X-YoloV5/best.pt'

    def __init__(self):
        super().__init__('detection_buoy')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', Detection.weights_path)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


        self.bbox_publisher = self.create_publisher(BboxMsg, "/camera/bbox") # publishes bounding box
        self.lidar_publisher = self.create_publisher(LidarBboxMsg, "/livox/bbox") # publishes bounding box in lidar coord


        self.load_intrinsic_distortion(Detection.intrinsic_path)
        self.load_extrinsic(Detection.extrinsic_path)

        self.width = 1448
        self.height = 568
        self.resize = 640

    def callback(self, detection_msg, lidar_msg):
        
        # Store bounding box coordinates. 
        bbox_x1 = detection_msg.x1
        bbox_x2 = detection_msg.x2


        n_pts = lidar_msg.point_num
        for i in range(n_pts):
            lidar_msg.points[i]


        # TODO
        pass


    def bbox_callback(self, data):
        img = data.data
        print(type(img))
        print(img)


    def data_callback(self):
        self.image_sub = self.create_subscription(Image, "/right_camera/image", self.bbox_callback, 10)
        # self.lidar_sub = self.create_subscription(LivoxPointMsg, "/livox/stamped", self.lidar_callback, 10)


    def my_callback(self, lidar_msg):
        # Convert bounding box points to lidar space
        # get all lidar points within that space
        # get median of X, Y, distance of those lidar points
        pass


    def __call__(self, image, label=None, plt_path=None):
        """
        Call image, label, plt_path
        """
        pass

    def _convert_coords(self, detections):
        # Detections = [class, bbox, ]
        pass



    def getDetection(image, plt_path=None):
        '''
        Given input image, return bounding box coordinates and class label.

        Returns
            Class labels: (b, )
                '0' - 'blue-buoy'
                '1' - 'pink-buoy'
                '2' - 'white-buoy'
            Box coordinates in image (1448, 563, 3): (b, x1, y1, x2, y2)
            Labeled image: (1448, 563, 3)
        '''

        # TODO: convert image to subscriber
        img = deepcopy(image)
        if image.shape == (self.width, self.height, 3): # resize
            img = cv2.resize(image, (self.resize, self.resize, 3))
        
        # TODO: make custom model
        detections = model(img).xyxyn[0] # n x 6

        msg = BboxMsg()

        msg.x1, msg.y1, msg.x2, msg.y2 = self.rescale_coordinates(detections[:, :4])
        msg.conf = detections[:, 4]
        msg.label = detections[:, -1]

        if plt_path:

            for pred in detections:
                x1, y1, x2, y2, confidence, label = pred
                label = str(int(label))
                x1, y1, x2, y2, cls = int(x1), int(y1), int(x2), int(y2), label2class[label]
                cv2.rectangle(img, (x1, y1), (x2, y2), label2color[label], 5)

            img = self.resize2original(img)
            plt.imshow(img)
            plt.savefig(plt_path)
            print(f'Saved predictions to {plt_path}')
            return msg, img

        return msg

    def _getLabels(label_path, plt_img):
        with open(label_path) as f:
            lines = [bbox.strip() for bbox in f.readlines()]
            labels = [l.split(' ') for l in lines]  # cls, cx, cy, w, h

        if plt_img:
            img = test_image.copy()
            for y in labels:
                label, cx, cy, w, h = y
                cls, cx, cy, w, h = label2class[label], scale(cx), scale(cy), scale(w), scale(h)
                cv2.rectangle(img, (int(cx - w/2), int(cy - h/2)), ((int(cx+w/2)), int(cy+h/2)), label2color[label], 5)
            return img

        return labels 

    def resize2original(self, img):
        return cv2.resize(img, (self.width, self.height))

    def rescale_coordinates(self, coords):
        # Coords: [x1, y1, x2, y2] - 0~1
        # Returns: [x1, y1, x2, y2] - 1448~568
        return coords * np.array([self.width, self.height, self.width, self.height])

    def load_intrinsic_distortion(self, intrinsic_path):
        intrinsic = np.zeros((3, 3))
        distortion = np.zeros((5, 1))
        with open(intrinsic_path, 'r') as f:
            lines = f.readlines()
            skip = [0, 4, 5]
            for i, line in enumerate(lines):
                if i in skip: 
                    continue
                line = line.strip().split(' ')
                if i < 4: # intrinsic
                    intrinsic[i-1] = [float(x)for x in line]
                else: # distortion
                    for j, dist in enumerate(line):
                        distortion[j] = float(dist)
        self.intrinsic = cp.asarray(intrinsic)
        self.distorition = cp.asarray(distorition)

    def load_extrinsic(self, extrinsic_path):
        extrinsic = np.zeros((4, 4))
        with open(extrinsic_path, 'r') as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if i == 0: 
                    continue
                line = line.split('  ')
                extrinsic[i-1] = [float(x) for x in line]

        self.extrin = cp.asarray(extrinsic[:3, :]) # 3x4
        self.extrinsic = cp.asarray(extrinsic) # 4x4

    def getTheoreticalUV(self, x, points):
        """
        Returns dist, U, V
        """
        # (3x3) x (3x4) x (4 x n)
        result = cp.matmul(cp.matmul(self.intrinsic, self.extrin), points.T)
        depth = result[2, :]
        u = result[0, :] / depth
        v = result[1, :] / depth
        u, v = cp.asnumpy(u), cp.asnumpy(v)
        x = np.array(x)

        return np.vstack((x, u, v))

def main(args=None):
    rclpy.init()
    node = Detection()
    node.data_callback()
    rclpy.spin(node)
    
if __name__ == '__main__':
        main()