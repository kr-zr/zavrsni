#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import os
import cv2
import random
from tqdm import tqdm
import tensorflow as tf
from tensorflow.keras import datasets, layers, models
from tensorflow.keras import callbacks
import time
from tensorflow.keras import backend as K
from tensorflow.python.keras import losses
from tensorflow.python.saved_model import builder as saved_model_builder
from tensorflow.python.saved_model import tag_constants, signature_constants
from tensorflow.python.saved_model.signature_def_utils_impl import predict_signature_def



IMG_SIZE = 80

CATEGORIES = ["ravno", "racvanje", "plod"]
DATADIR_TEST = "/home/karlo/biljka/testing"

testing_data = []

def create_testing_data ():
    for category in CATEGORIES:

        path_test = os.path.join(DATADIR_TEST, category)

        for img in tqdm(os.listdir(path_test)):
            try:
                img_array_test = cv2.imread(os.path.join(path_test,img),cv2.IMREAD_GRAYSCALE)
                new_array_test = cv2.resize(img_array_test, (IMG_SIZE, IMG_SIZE))
                testing_data.append(new_array_test)
            except Exception as e:
                pass

create_testing_data()

random.shuffle(testing_data)

plt.imshow(testing_data[0],cmap='gray')
plt.show()

testing_data = np.array(testing_data).reshape(-1, IMG_SIZE,IMG_SIZE,1)
#print(testing_data[0])

testing_data = testing_data/255.0

model = tf.keras.models.load_model("treci_strojno",compile=False)  #ValueError: Unknown entries in loss dictionary: [u'class_name', u'config']. Only expected following keys: [u'dense_1']

model_optimizer = tf.keras.optimizers.Adam(learning_rate=0.00007)
model.compile(optimizer=model_optimizer, loss=tf.keras.losses.CategoricalCrossentropy(from_logits=True), metrics=['accuracy'])

predictions = model.predict_classes([testing_data])
print(CATEGORIES[predictions[0]])

