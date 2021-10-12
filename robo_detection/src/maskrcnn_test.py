from maskrcnn_detect import MaskRcnn
import cv2, os
import numpy as np
import signal
import sys

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

currentdir = os.path.dirname(os.path.realpath(__file__))
print("Loading MaskRCNN mdoel...")
test_img = cv2.imread(currentdir+"/test/2.jpg")

# test_img = cv2.imread("/home/bowen-robolution/Downloads/11.png")
masktest = MaskRcnn(currentdir+"/maskrcnn_weights/model_final_f10217.pkl")
# img, mask = masktest.inference(test_img)
img, mask_rtn = masktest.inference(test_img, [0], 3, 0.1)
if mask_rtn is not None:
    cv2.imshow("A", img)
    cv2.imshow("B", mask_rtn)
    cv2.waitKey(0)
    img_shape = img.shape[:2]
    uv = np.random.randint(2000, size=(2, 9984)) - 1000
    p_xyz = np.random.random((3, 9984)) * 1000
    uv_within_frame_ind = np.where((uv[1, :]>0) & (uv[1, :]<img_shape[0]) & (uv[0, :]>0) & (uv[0, :]<img_shape[1]))[0] # uv[0, :] => x
    ok_list = []
    for point_ind in uv_within_frame_ind.tolist():
        # print(uv[:, point_ind])
        u, v = uv[:, point_ind][0], uv[:, point_ind][1]
        if mask_rtn[v, u]:
            ok_list.append(point_ind)
    ok_array = np.array(ok_list)
    print(ok_array)
    print(p_xyz[:, ok_array])
    
# a = np.array()