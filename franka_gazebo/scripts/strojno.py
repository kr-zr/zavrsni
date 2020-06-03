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
from tensorflow.keras import backend

def give_label (path):

    if "ravno" in path:
        label = [1,0,0]
    if "racvanje" in path:
        label = [0,1,0]
    if "plod" in path:
        label = [0,0,1]
    return label

NAME = "dijelovi stabljike biljke"
DATADIR = "/home/karlo/biljka"
CATEGORIES = ["ravno", "racvanje", "plod"]
DATADIR_TEST = "/home/karlo/biljka/testing"

#for category in CATEGORIES:
    #path = os.path.join (DATADIR, category)
    #for img in os.listdir(path):
        #img_array = cv2.imread (os.path.join(path,img), cv2.IMREAD_GRAYSCALE)
        #plt.imshow (img_array , cmap = 'gray')
        #plt.show()
        #break
IMG_SIZE = 80

#new_array = cv2.resize (img_array, (IMG_SIZE,IMG_SIZE))
#plt.imshow (new_array, cmap = 'gray')
#plt.show()
training_data = []
testing_data = []
def create_training_data ():
    for category in CATEGORIES:

        path = os.path.join(DATADIR, category)
        class_label = give_label(path)

        for img in tqdm(os.listdir(path)):
            try:
                img_array = cv2.imread(os.path.join(path,img),cv2.IMREAD_GRAYSCALE)
                new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))
                training_data.append([new_array, class_label])
            except Exception as e:
                pass
def create_testing_data ():
    for category in CATEGORIES:

        path_test = os.path.join(DATADIR_TEST, category)
        class_label_test = give_label(path_test)

        for img in tqdm(os.listdir(path_test)):
            try:
                img_array_test = cv2.imread(os.path.join(path_test,img),cv2.IMREAD_GRAYSCALE)
                new_array_test = cv2.resize(img_array_test, (IMG_SIZE, IMG_SIZE))
                testing_data.append([new_array_test, class_label_test])
            except Exception as e:
                pass
create_training_data()
create_testing_data()
#print(len(testing_data))
random.shuffle(training_data)
random.shuffle(testing_data)
#for sample in training_data[:10]:
    #print(sample[1])
    #plt.imshow (sample[0], cmap = 'gray')
    #plt.show()
X = []
Y = []
x_test = []
y_test = []
for features, label in training_data:
    X.append(features)
    Y.append(label)
for features, label in testing_data:
    x_test.append(features)
    y_test.append(label)
print(y_test[0])
#plt.imshow(x_test[0],cmap='gray')
#plt.show()
x = np.array(X).reshape (-1, IMG_SIZE,IMG_SIZE,1)
x = x/255.0
Y = np.array(Y)
x_test = np.array(x_test).reshape (-1, IMG_SIZE,IMG_SIZE,1)
x_test= x_test/255.0
y_test = np.array(y_test)
model = models.Sequential()
model.add(layers.Conv2D(64, (3, 3), activation='relu', input_shape=x.shape[1:]))
model.add(layers.MaxPooling2D((3, 3)))
model.add(layers.Conv2D(32, (3, 3), activation='relu'))
model.add(layers.MaxPooling2D((3, 3)))
model.add(layers.Conv2D(16, (3, 3), activation='relu'))
#model.add(layers.MaxPooling2D((2, 2)))
#model.add(layers.Conv2D(64, (3, 3), activation='relu'))
#model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Flatten())
#model.add(layers.Dense(128,activation = 'relu'))
#model.add(layers.Dropout(0.9))
model.add(layers.Dense(512))
model.add(layers.Dropout(0.9))
#model.add(layers.Dense(8))
#model.add(layers.Dropout(0.9))
model.add(layers.Dense(3,activation = 'relu'))
model_optimizer = tf.keras.optimizers.Adam(learning_rate=0.000075)
#tensorboard = callbacks.TensorBoard(log_dir="logs/{}".format(NAME))
model.compile(optimizer=model_optimizer, loss=tf.keras.losses.CategoricalCrossentropy(from_logits = True), metrics=['accuracy'])
model.fit (x,Y,batch_size = 20,epochs = 65,validation_split = 0.1, validation_data = (x_test,y_test))
predictions = model.predict_classes([x_test])
print(CATEGORIES[predictions[0]])
#plt.imshow(x_test[0])
