import json
import os
import time

import cv2

# Grounding DINO
import groundingdino.datasets.transforms as T
import matplotlib.pyplot as plt
import numpy as np
import PIL.Image as PIL_Image
import torch
from groundingdino.models import build_model
from groundingdino.util.slconfig import SLConfig
from groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap

# segment anything
from segment_anything import SamPredictor, sam_model_registry  # sam_hq_model_registry


class grounded_sam:
    def __init__(self):
        # self.args = args
        # cfg
        # print("pwd: ", os.getcwd())
        script_dir = os.path.dirname(__file__)
        root_dir = os.path.join(script_dir, "../../../")
        gsam_dir = os.path.join(script_dir, "../../../../GroundingDINO")

        self.config_file = os.path.join(
            gsam_dir, "groundingdino/config/GroundingDINO_SwinT_OGC.py"
        )
        self.grounded_checkpoint = os.path.join(
            root_dir, "assets/weights/groundingdino_swint_ogc.pth"
        )

        # self.config_file = os.path.join(
        #     root_dir, "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
        # )  # change the path of the model config file
        # self.grounded_checkpoint = os.path.join(
        #     root_dir, "groundingdino_swint_ogc.pth"  # change the path of the model
        # )
        self.sam_version = "vit_h"
        self.sam_checkpoint = os.path.join(
            root_dir, "assets/weights/sam_vit_h_4b8939.pth"
        )
        self.sam_hq_checkpoint = None
        self.use_sam_hq = False
        # image_path = args.input_image
        self.text_prompt = None
        self.output_dir = os.path.join(root_dir, "output")
        self.box_threshold = 0.3
        self.text_threshold = 0.25
        self.device = "cuda"
        self.save_output = True

        # make dir
        # os.makedirs(self.output_dir, exist_ok=True)
        # load image
        # image_pil, image = self.load_image(image_path)

        # load model DINO
        self.model = self.load_model(
            self.config_file, self.grounded_checkpoint, device=self.device
        )

        # initialize SAM
        self.predictor = None
        if self.use_sam_hq:
            self.predictor = SamPredictor(
                sam_hq_model_registry[self.sam_version](
                    checkpoint=self.sam_hq_checkpoint
                ).to(self.device)
            )
        else:
            self.predictor = SamPredictor(
                sam_model_registry[self.sam_version](checkpoint=self.sam_checkpoint).to(
                    self.device
                )
            )

    def get_masks(self, prompts, image):
        with torch.no_grad():
            # rgb = bgr[...,::-1].copy()
            # bgr = rgb[...,::-1].copy()
            # gbr = rgb[...,[2,0,1]].copy()
            masks_output = []

            for prompt in prompts:
                image_np = np.array(image)
                image_pil, image_tensor = self.process_image(image_np)
                start = time.time()
                boxes_filt, pred_phrases, pred_probs = self.get_grounding_output(
                    self.model,
                    image_tensor,
                    prompt,
                    self.box_threshold,
                    self.text_threshold,
                    device=self.device,
                )
                end = time.time()
                print("run grounding dino model time: ", end - start, " s")

                # image = cv2.imread(image_path)
                # print("image2: ",image.shape())
                # image = rgb_img # RGB img
                start = time.time()
                self.predictor.set_image(image)

                size = image_pil.size
                H, W = size[1], size[0]
                for i in range(boxes_filt.size(0)):
                    boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])
                    boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
                    boxes_filt[i][2:] += boxes_filt[i][:2]

                boxes_filt = boxes_filt.cpu()
                transformed_boxes = self.predictor.transform.apply_boxes_torch(
                    boxes_filt, image.shape[:2]
                ).to(self.device)

                masks, _, _ = self.predictor.predict_torch(
                    point_coords=None,
                    point_labels=None,
                    boxes=transformed_boxes.to(self.device),
                    multimask_output=False,
                )
                # Append the max prob mask
                # Sort the masks by the max prob
                max_prob = np.argsort(pred_probs)[::-1]
                masks = masks.cpu().numpy()[max_prob]
                masks_output.append(masks)
                end = time.time()
                print("SAM time: ", end - start, " s")

                # draw output image
                if self.save_output:
                    # plt.figure(figsize=(10, 10))
                    # plt.imshow(image)
                    # for mask in masks_output:
                    #     self.show_mask(mask[0], plt.gca(), random_color=True)
                    # for box, label in zip(boxes_filt, pred_phrases):
                    #     self.show_box(box.numpy(), plt.gca(), label)

                    # plt.axis("off")
                    # plt.savefig(
                    #     os.path.join(
                    #         self.output_dir, f"grounded_sam_output{prompt}.jpg"
                    #     ),
                    #     bbox_inches="tight",
                    #     dpi=300,
                    #     pad_inches=0.0,
                    # )
                    self.save_mask_data(
                        self.output_dir, masks, boxes_filt, pred_phrases
                    )

            return masks_output

    def prostprocess_masks(self, masks):
        """
        To each mask, apply morphological operations to remove small noise. The
        size of the kernel should be proportional to the size of the mask.
        """
        processed_masks = []
        for mask in masks:
            area = mask.sum()
            kernel_size = max(1, (area) // 10000)
            mask = cv2.morphologyEx(
                mask.astype(np.uint8),
                cv2.MORPH_CLOSE,
                np.ones((kernel_size, kernel_size), np.uint8),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            processed_masks.append(mask)
        processed_masks = np.array(processed_masks, dtype=bool)
        return processed_masks

    def show_mask(self, mask, ax, random_color=False):
        if random_color:
            color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
        else:
            color = np.array([30 / 255, 144 / 255, 255 / 255, 0.6])
        h, w = mask.shape[-2:]
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        ax.imshow(mask_image)

    def show_box(self, box, ax, label):
        x0, y0 = box[0], box[1]
        w, h = box[2] - box[0], box[3] - box[1]
        ax.add_patch(
            plt.Rectangle(
                (x0, y0), w, h, edgecolor="green", facecolor=(0, 0, 0, 0), lw=2
            )
        )
        ax.text(x0, y0, label)

    def save_mask_data(self, output_dir, mask_list, box_list, label_list):
        value = 0  # 0 for background

        mask_img = torch.zeros(mask_list.shape[-2:])
        for idx, mask in enumerate(mask_list):
            mask_img[mask[0] == True] = value + idx + 1
            # print( mask.cpu().numpy().shape)
        # plt.figure(figsize=(10, 10))
        # plt.imshow(mask_img.numpy())
        # plt.axis("off")
        # plt.savefig(
        #     os.path.join(output_dir, "mask.jpg"),
        #     bbox_inches="tight",
        #     dpi=300,
        #     pad_inches=0.0,
        # )

        json_data = [{"value": value, "label": "background"}]
        for label, box in zip(label_list, box_list):
            value += 1
            name, logit = label.split("(")
            logit = logit[:-1]  # the last is ')'
            json_data.append(
                {
                    "value": value,
                    "label": name,
                    "logit": float(logit),
                    "box": box.numpy().tolist(),
                }
            )
        with open(os.path.join(output_dir, "mask.json"), "w") as f:
            json.dump(json_data, f)

    def load_image(self, image_path):
        # load image
        image_pil = PIL_Image.open(image_path).convert("RGB")  # load image

        transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        image, _ = transform(image_pil, None)  # 3, h, w
        return image_pil, image

    def process_image(self, rgb_img):
        image_pil = PIL_Image.fromarray(rgb_img.astype("uint8"), "RGB")
        transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        image, _ = transform(image_pil, None)  # 3, h, w
        return image_pil, image

    def load_model(self, model_config_path, model_checkpoint_path, device):
        args = SLConfig.fromfile(model_config_path)
        args.device = device
        model = build_model(args)
        checkpoint = torch.load(model_checkpoint_path, map_location="cpu")
        load_res = model.load_state_dict(
            clean_state_dict(checkpoint["model"]), strict=False
        )
        print(load_res)
        _ = model.eval()
        return model

    def get_grounding_output(
        self,
        model,
        image,
        caption,
        box_threshold,
        text_threshold,
        with_logits=True,
        device="cpu",
    ):
        caption = caption.lower()
        caption = caption.strip()
        if not caption.endswith("."):
            caption = caption + "."
        model = model.to(device)
        image = image.to(device)
        with torch.no_grad():
            outputs = model(image[None], captions=[caption])
        logits = outputs["pred_logits"].cpu().sigmoid()[0]  # (nq, 256)
        boxes = outputs["pred_boxes"].cpu()[0]  # (nq, 4)
        logits.shape[0]

        # filter output
        logits_filt = logits.clone()
        boxes_filt = boxes.clone()
        filt_mask = logits_filt.max(dim=1)[0] > box_threshold
        logits_filt = logits_filt[filt_mask]  # num_filt, 256
        boxes_filt = boxes_filt[filt_mask]  # num_filt, 4
        logits_filt.shape[0]

        # get phrase
        tokenlizer = model.tokenizer
        tokenized = tokenlizer(caption)
        # build pred
        pred_phrases = []
        pred_probs = []
        for logit, box in zip(logits_filt, boxes_filt):
            pred_phrase = get_phrases_from_posmap(
                logit > text_threshold, tokenized, tokenlizer
            )
            if with_logits:
                pred_phrases.append(pred_phrase + f"({str(logit.max().item())[:4]})")
                pred_probs.append(logit.max())
            else:
                pred_phrases.append(pred_phrase)

        return boxes_filt, pred_phrases, pred_probs

    def get_mask_bbox(self, mask):
        """
        Given a mask, return the bounding box of the mask
        in the format (x_min, x_max, y_min, y_max)
        """
        # masks: masks
        # return: x, y limits of the mask
        x_min, x_max = np.where(mask)[1].min(), np.where(mask)[1].max()
        y_min, y_max = np.where(mask)[0].min(), np.where(mask)[0].max()

        margin = (x_max - x_min) // 100
        x_min = max(0, x_min - margin)
        x_max = min(mask.shape[1], x_max + margin)
        y_min = max(0, y_min - margin)
        y_max = min(mask.shape[0], y_max + margin)

        return x_min, x_max, y_min, y_max


def demo():
    image_path = "assets/IMG_1028.jpg"

    gsam = grounded_sam()
    image_pil, image = gsam.load_image(image_path)
    image = cv2.imread(image_path)
    masks = gsam.get_masks(["mug", "microwave door"], image)
    # masks = gsam.get_masks( ["mug", "microwave door"], image)


if __name__ == "__main__":
    demo()
