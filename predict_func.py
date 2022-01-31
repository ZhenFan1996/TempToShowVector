

import torch
from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from pathlib import Path
import cv2

DEVICE = "cpu"
MODEL_PATH = "./model/best2.pt"


@torch.no_grad()
def model_load(weights="",  # model.pt path(s)
               device=DEVICE,  # cuda device, i.e. 0 or 0,1,2,3 or cpu
               half=False,  # use FP16 half-precision inference
               dnn=False,  # use OpenCV DNN for ONNX inference
               ):
    device = select_device(device)
    half &= device.type != 'cpu'  # half precision only supported on CUDA
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx = model.stride, model.names, model.pt, model.jit, model.onnx
    # Half
    half &= pt and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
    if pt:
        model.model.half() if half else model.model.float()
    return model


def detect_img(model, img_path):
    model = model
    hide_labels = False  # hide labels
    hide_conf = False
    save_img = True,
    source = img_path  # file/dir/URL/glob, 0 for webcam
    imgsz = [640, 640]  # inference size (pixels)
    output_size = 640
    conf_thres = 0.25  # confidence threshold
    iou_thres = 0.45  # NMS IOU threshold
    max_det = 1000  # maximum detections per image
    device = DEVICE  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    save_conf = False  # save confidences in --save-txt labels
    save_crop = False  # save cropped prediction boxes
    nosave = False  # do not save images/videos
    classes = None  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms = False  # class-agnostic NMS
    augment = False  # ugmented inference
    visualize = False  # visualize features
    line_thickness = 3  # bounding box thickness (pixels)
    half = False  # use FP16 half-precision inference
    source = str(source)
    device = select_device(device)
    webcam = False
    stride, names, pt, jit, onnx = model.stride, model.names, model.pt, model.jit, model.onnx
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt and not jit)
    bs = 1  # batch_size
    # Run inference
    if pt and device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.model.parameters())))  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(device)
        im = im.half() if half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1
        # Inference
        pred = model(im, augment=augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2
        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3
        final_results = []
        # print(pred)
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
            p = Path(p)  # to Path
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(
                        -1).tolist()  # normalized xywh
                    # line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                    # location = (*xyxy)
                    # print(xyxy)
                    # line = (cls, *xywh, conf)
                    if save_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))

                    cls_name = names[int(cls.cpu().numpy())]
                    location = [x.cpu().numpy().item() for x in xyxy]
                    conf_score = conf.cpu().numpy()
                    result = {"class_name": cls_name,
                              "xmin": int(location[0]),
                              "ymin": int(location[1]),
                              "xmax": int(location[2]),
                              "ymax:": int(location[3]),
                              "score": float(conf_score)}
                    final_results.append(result)

            im0 = annotator.result()
            frame = im0
            resize_scale = output_size / frame.shape[0]
            frame_resized = cv2.resize(frame, (0, 0), fx=resize_scale, fy=resize_scale)
            cv2.imwrite("single_result_vid.jpg", frame_resized)
        return final_results,frame_resized
        # return {"results": final_results}

model = model_load(weights=MODEL_PATH,device=DEVICE)

def run(img):

     results,frame = detect_img(model,img)
     return results,frame


#if __name__ == "__main__":
#    # 指明模型路径
#    model = model_load(weights=MODEL_PATH,
#                       device=DEVICE)  # todo 指明模型加载的位置的设备
#    results = detect_img(model=model, img_path="data/images/bus.jpg")
#    print(results)
