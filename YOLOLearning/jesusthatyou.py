import cv2
import os
import sys
import pyzed.sl as sl
from ultralytics import YOLO
import supervision as sv

byte_tracker = sv.ByteTrack()
box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator()
line_annotator = sv.LineZoneAnnotator(thickness=2, text_thickness=2, text_scale=0.5)

# Load a YOLOv8 model
trt_model = YOLO("/home/xplanter/YOLOLearning/runs/detect/train7/weights/best.pt")

# Export the model to TensorRT with DLA enabled (only works with FP16 or INT8)
# model.export(format="engine", device="dla:0", half=True)  # dla:0 or dla:1 corresponds to the DLA cores

# # Load the exported TensorRT model
# trt_model = YOLO("yolo11n.engine")

zed = sl.Camera()

input_type = sl.InputType()
if len(sys.argv) >= 2:
    input_type.set_from_svo_file(sys.argv[1])
init = sl.InitParameters(input_t=input_type)
init.camera_resolution = sl.RESOLUTION.HD1080
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init.coordinate_units = sl.UNIT.MILLIMETER

err = zed.open(init)
if err != sl.ERROR_CODE.SUCCESS:
    print(repr(err))
    zed.close()
    exit(1)

runtime = sl.RuntimeParameters()

# Prepare new image size to retrieve half-resolution images
image_size = zed.get_camera_information().camera_configuration.resolution
image_size.width = image_size.width / 2
image_size.height = image_size.height / 2

# Declare your sl.Mat matrices
image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

line_zone = sv.LineZone(sv.Point(0, image_size.height / 2), sv.Point(image_size.width, image_size.height / 2))

healthy_plants = 0
unhealthy_plants = 0

# Define plant health detection logic (You can adjust based on model outputs)
def classify_health(detection):
    # Example: Assuming that 'class_id' of the detected plant is 0 for healthy and 1 for unhealthy
    # Replace this logic with actual health classification based on the model's output
    if detection[4] == 0:  # Healthy plant
        return "healthy"
    else:  # Unhealthy plant
        return "unhealthy"

# Now pass the frame_rgb to the model
while True:
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        # Retrieve the left image, depth image in the half-resolution
        zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
        frame = image_zed.get_data()
        # Before passing the frame to the model, convert it to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        frame_gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

        # Now pass the frame_rgb to the model
        results = trt_model(frame_gray)

        annotated_image = results[0].plot()

        # Display the annotated image using OpenCV
        cv2.imshow("Detection Results", annotated_image)
    if cv2.waitKey(1) == ord('q'):
        break
# results = trt_model("/home/xplanter/YOLOLearning/TomatoSeedlingDataset/test/images/image_836_png.rf.2759786abe94961b5327a5b5bcccb1d3.jpg")


zed.close()

print("\nFINISH")
