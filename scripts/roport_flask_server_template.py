#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Common utilities
import cv2
import json
import base64
import waitress
import numpy as np

# Frameworks
import torch

# Flask
from flask import Flask, request, make_response

# Custom functions
# TODO add this

'''
Note that this is a template for guiding the usage of Flask server
for porting the algorithm using Python3. You could implement the
server for your algorithm by referring this.
'''

app = Flask(__name__, template_folder="templates")


def get_results(cv_image_list):
    """Get the results using the algorithm"""
    results = []
    return results


def decode_b64_to_image(b64_str, is_bgr=True):
    """Decode the base64 string as OpenCV image, the b64_str is produced
    by encode_image_to_b64 with BGR or depth image.

    :param b64_str: str base64 string produced by encode_image_to_b64
    :param is_bgr: bool If true, the string is treated as BGR (8UC3), otherwise depth (16UC1)
    :return: ok, cv2_image
    """
    if "," in b64_str:
        b64_str = b64_str.partition(",")[-1]
    else:
        b64_str = b64_str

    try:
        img = base64.b64decode(b64_str)
        # imdecode use the same flag as imread. cf. https://docs.opencv.org/3.4/d8/d6a/group__imgcodecs__flags.html
        if is_bgr:
            return True, cv2.imdecode(np.frombuffer(img, dtype=np.uint8), cv2.IMREAD_COLOR)
        else:
            return True, cv2.imdecode(np.frombuffer(img, dtype=np.uint8), cv2.IMREAD_ANYDEPTH)
    except cv2.error:
        return False, None


@app.route('/process', methods=['POST'])
def process():
    """The main entrance of the server.

    Input: images
    Output: status (bool), results (JSON array)
    """
    try:
        req_data = json.loads(request.data)
        if 'body' not in req_data:
            return make_response("Failed to load request data", 200)
        else:
            req_body = req_data['body']
    except ValueError:
        return make_response("Failed to phase JSON", 200)

    header = {}
    response = {'status': False, 'results': []}

    cv_image_list = []
    image_strings = json.loads(req_body['images'])
    for s in image_strings:
        ok, image = decode_b64_to_image(s)
        if not ok:
            feedback = {'header': header, 'response': response}
            return make_response(json.dumps(feedback), 200)
        cv_image_list.append(image)

    ok, results = get_results(cv_image_list)
    torch.cuda.empty_cache()

    if ok:
        response['status'] = True
        response['results'] = json.dumps(results)
    feedback = {'header': header, 'response': response}
    return make_response(json.dumps(feedback), 200)


if __name__ == "__main__":
    waitress.serve(app, host='0.0.0.0', port=6060, threads=6)
