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
trt_model = YOLO("/home/xplanter/YOLOLearning/runs/detect/train11/weights/best.pt")

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

while True:
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        # Retrieve the left image, depth image in the half-resolution
        zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
        frame = image_zed.get_data()
        # Before passing the frame to the model, convert it to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        # Now pass the frame_rgb to the model
        results = trt_model.predict(frame_rgb, save=True, iou = 0.35, conf=0.3)
        print(f"Results: {results[0]}")
                
        detections = sv.Detections.from_ultralytics(results[0])
        detections = byte_tracker.update_with_detections(detections)

        print(detections)

        # Assuming model is your YOLO model and detections have been processed
        labels = [
            f"#{tracker_id} {trt_model.names[class_id]}"
            for class_id, tracker_id in zip(detections.class_id, detections.tracker_id)
        ]


        
        cross_in, cross_out = line_zone.trigger(detections)

        print(detections)
        
        # print(cross_in)
        # # Track healthy vs. unhealthy plants
        # for detection in detections:
        #     print(detection)
        #     plant_health = classify_health(detection)

        #     print(cross_in)
        #     if plant_health == "healthy":
        #         healthy_plants += cross_in[detection[5]]
        #         healthy_plants -= cross_out[detection[5]]
        #     else:
        #         unhealthy_plants += cross_in[detection[5]]
        #         unhealthy_plants -= cross_out[detection[5]]

        frame_rgb = box_annotator.annotate(scene=frame_rgb, detections=detections)
        frame_rgb = line_annotator.annotate(frame_rgb, line_zone)
        frame_rgb = label_annotator.annotate(frame_rgb, detections = detections, labels=labels)
        
        cv2.imshow("cam", frame_rgb)
        
        print(f"Healthy plants: {healthy_plants}, Unhealthy plants: {unhealthy_plants}")

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
zed.close()

print("\nFINISH")
